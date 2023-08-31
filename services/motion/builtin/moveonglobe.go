package builtin

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	goutils "go.viam.com/utils"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

type execManager struct {
	plan                   motionplan.Plan
	successChan            chan bool
	positionPollingErrChan chan error
	obsticlePollingErrChan chan error
	executionErrChan       chan error
	replanChan             chan bool
	cancelCtx              context.Context
	cancelFn               context.CancelFunc
	backgroundWorkers      *sync.WaitGroup
}

func plan(
	ctx context.Context,
	planRequest *motionplan.PlanRequest,
	kb kinematicbase.KinematicBase,
	componentName string,
) (motionplan.Plan, error) {
	inputs, err := kb.CurrentInputs(ctx)
	if err != nil {
		return make(motionplan.Plan, 0), err
	}
	// TODO: this is really hacky and we should figure out a better place to store this information
	if len(kb.Kinematics().DoF()) == 2 {
		inputs = inputs[:2]
	}
	planRequest.StartConfiguration = map[string][]referenceframe.Input{componentName: inputs}

	return motionplan.PlanMotion(ctx, planRequest)
}

func startPollingForReplan(
	ctx context.Context,
	period time.Duration,
	errChan chan error,
	replanChan chan bool,
	doneFn func(),
	fn func(context.Context,
	) (bool, error),
) {
	goutils.ManagedGo(func() {
		ticker := time.NewTicker(period)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				replan, err := fn(ctx)
				if err != nil {
					errChan <- err
					return
				}
				if replan {
					// TODO: the function that actually monitors obstacles
					replanChan <- replan
					return
				}
			}
		}
	}, doneFn)
}

func spawnExecute(
	motionCfg *motion.MotionConfiguration,
	kb kinematicbase.KinematicBase,
	ms *builtIn,
	movementSensor movementsensor.MovementSensor,
	destination *geo.Point,
	manager *execManager,
) {
	positionPollingPeriod := time.Duration(1000/motionCfg.PositionPollingFreqHz) * time.Millisecond
	obstaclePollingPeriod := time.Duration(1000/motionCfg.ObstaclePollingFreqHz) * time.Millisecond

	// helper function to manage polling functions
	// spawn two goroutines that each have the ability to trigger a replan
	if positionPollingPeriod > 0 {
		manager.backgroundWorkers.Add(1)
		startPollingForReplan(
			manager.cancelCtx,
			positionPollingPeriod,
			manager.positionPollingErrChan,
			manager.replanChan,
			manager.backgroundWorkers.Done,
			func(ctx context.Context) (bool, error) {
				return false, nil
			})
	}
	if obstaclePollingPeriod > 0 {
		manager.backgroundWorkers.Add(1)
		startPollingForReplan(
			manager.cancelCtx,
			obstaclePollingPeriod,
			manager.obsticlePollingErrChan,
			manager.replanChan,
			manager.backgroundWorkers.Done,
			func(ctx context.Context,
			) (bool, error) {
				return true, nil
			})
	}

	// spawn function to execute the plan on the robot
	manager.backgroundWorkers.Add(1)
	goutils.ManagedGo(func() {
		if err := ms.execute(manager.cancelCtx, kb, manager.plan); err != nil {
			manager.executionErrChan <- err
			return
		}

		// the plan has been fully executed so check to see if the GeoPoint we are at is close enough to the goal.
		success, err := arrivedAtGoal(
			manager.cancelCtx,
			movementSensor,
			destination,
			motionCfg.PlanDeviationMM,
		)
		if err != nil {
			manager.executionErrChan <- err
			return
		}

		if success {
			manager.successChan <- success
			return
		}
		manager.executionErrChan <- errors.New("failed to arrive at goal")
	}, manager.backgroundWorkers.Done)
}

func arrivedAtGoal(ctx context.Context, ms movementsensor.MovementSensor, destination *geo.Point, radiusMM float64) (bool, error) {
	position, _, err := ms.Position(ctx, nil)
	if err != nil {
		return false, err
	}
	if spatialmath.GeoPointToPose(position, destination).Point().Norm() <= radiusMM {
		return true, nil
	}
	return false, nil
}

func newExecutionManager(ctx context.Context, plan motionplan.Plan) *execManager {
	cancelCtx, cancelFn := context.WithCancel(ctx)

	var backgroundWorkers sync.WaitGroup
	defer backgroundWorkers.Wait()

	return &execManager{
		plan:                   plan,
		successChan:            make(chan bool, 1),
		replanChan:             make(chan bool, 1),
		executionErrChan:       make(chan error, 1),
		obsticlePollingErrChan: make(chan error, 1),
		positionPollingErrChan: make(chan error, 1),
		cancelCtx:              cancelCtx,
		cancelFn:               cancelFn,
		backgroundWorkers:      &backgroundWorkers,
	}
}

func flushChan[T any](c chan T) {
	for i := 0; i < len(c); i++ {
		<-c
	}
}

func (s *execManager) flush() {
	s.cancelFn()
	flushChan(s.successChan)
	flushChan(s.replanChan)
	flushChan(s.executionErrChan)
	flushChan(s.obsticlePollingErrChan)
	flushChan(s.positionPollingErrChan)
	s.backgroundWorkers.Wait()
}

func (ms *builtIn) newMoveOnGlobeRequest(
	ctx context.Context,
	componentName resource.Name,
	destination *geo.Point,
	movementSensor movementsensor.MovementSensor,
	obstacles []*spatialmath.GeoObstacle,
	motionCfg *motion.MotionConfiguration,
	extra map[string]interface{},
) (*motionplan.PlanRequest, kinematicbase.KinematicBase, error) {
	kinematicsOptions := kinematicbase.NewKinematicBaseOptions()
	if motionCfg.LinearMPerSec != 0 {
		kinematicsOptions.LinearVelocityMMPerSec = motionCfg.LinearMPerSec * 1000
	}
	if motionCfg.AngularDegsPerSec != 0 {
		kinematicsOptions.AngularVelocityDegsPerSec = motionCfg.AngularDegsPerSec
	}
	if motionCfg.PlanDeviationMM != 0 {
		kinematicsOptions.PlanDeviationThresholdMM = motionCfg.PlanDeviationMM
	}
	kinematicsOptions.GoalRadiusMM = motionCfg.PlanDeviationMM
	kinematicsOptions.HeadingThresholdDegrees = 8

	// build the localizer from the movement sensor
	origin, _, err := movementSensor.Position(ctx, nil)
	if err != nil {
		return nil, nil, err
	}

	// add an offset between the movement sensor and the base if it is applicable
	baseOrigin := referenceframe.NewPoseInFrame(componentName.ShortName(), spatialmath.NewZeroPose())
	movementSensorToBase, err := ms.fsService.TransformPose(ctx, baseOrigin, movementSensor.Name().ShortName(), nil)
	if err != nil {
		movementSensorToBase = baseOrigin
	}
	localizer := motion.NewMovementSensorLocalizer(movementSensor, origin, movementSensorToBase.Pose())

	// convert destination into spatialmath.Pose with respect to where the localizer was initialized
	goal := spatialmath.GeoPointToPose(destination, origin)

	// convert GeoObstacles into GeometriesInFrame with respect to the base's starting point
	geoms := spatialmath.GeoObstaclesToGeometries(obstacles, origin)

	gif := referenceframe.NewGeometriesInFrame(referenceframe.World, geoms)
	worldState, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{gif}, nil)
	if err != nil {
		return nil, nil, err
	}

	// construct limits
	straightlineDistance := goal.Point().Norm()
	if straightlineDistance > maxTravelDistance {
		return nil, nil, fmt.Errorf("cannot move more than %d kilometers", int(maxTravelDistance*1e-6))
	}
	limits := []referenceframe.Limit{
		{Min: -straightlineDistance * 3, Max: straightlineDistance * 3},
		{Min: -straightlineDistance * 3, Max: straightlineDistance * 3},
		{Min: -2 * math.Pi, Max: 2 * math.Pi},
	}

	if extra != nil {
		if profile, ok := extra["motion_profile"]; ok {
			motionProfile, ok := profile.(string)
			if !ok {
				return nil, nil, errors.New("could not interpret motion_profile field as string")
			}
			kinematicsOptions.PositionOnlyMode = motionProfile == motionplan.PositionOnlyMotionProfile
		}
	}
	ms.logger.Debugf("base limits: %v", limits)

	// create a KinematicBase from the componentName
	baseComponent, ok := ms.components[componentName]
	if !ok {
		return nil, nil, resource.NewNotFoundError(componentName)
	}
	b, ok := baseComponent.(base.Base)
	if !ok {
		return nil, nil, fmt.Errorf("cannot move component of type %T because it is not a Base", baseComponent)
	}

	kb, err := kinematicbase.WrapWithKinematics(ctx, b, ms.logger, localizer, limits, kinematicsOptions)
	if err != nil {
		return nil, nil, err
	}

	// create a new empty framesystem which we add the kinematic base to
	fs := referenceframe.NewEmptyFrameSystem("")
	kbf := kb.Kinematics()
	if err := fs.AddFrame(kbf, fs.World()); err != nil {
		return nil, nil, err
	}

	// TODO(RSDK-3407): this does not adequately account for geometries right now since it is a transformation after the fact.
	// This is probably acceptable for the time being, but long term the construction of the frame system for the kinematic base should
	// be moved under the purview of the kinematic base wrapper instead of being done here.
	offsetFrame, err := referenceframe.NewStaticFrame("offset", movementSensorToBase.Pose())
	if err != nil {
		return nil, nil, err
	}
	if err := fs.AddFrame(offsetFrame, kbf); err != nil {
		return nil, nil, err
	}

	return &motionplan.PlanRequest{
		Logger:             ms.logger,
		Goal:               referenceframe.NewPoseInFrame(referenceframe.World, goal),
		Frame:              offsetFrame,
		FrameSystem:        fs,
		StartConfiguration: referenceframe.StartPositions(fs),
		WorldState:         worldState,
		Options:            extra,
	}, kb, nil
}
