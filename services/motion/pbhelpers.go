package motion

import (
	"errors"
	"math"

	"github.com/google/uuid"
	geo "github.com/kellydunn/golang-geo"
	pb "go.viam.com/api/service/motion/v1"

	rprotoutils "go.viam.com/rdk/protoutils"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// planWithStatusFromProto converts a *pb.PlanWithStatus to a PlanWithStatus.
func planWithStatusFromProto(pws *pb.PlanWithStatus) (PlanWithStatus, error) {
	if pws == nil {
		return PlanWithStatus{}, errors.New("received nil *pb.PlanWithStatus")
	}

	plan, err := planFromProto(pws.Plan)
	if err != nil {
		return PlanWithStatus{}, err
	}

	status, err := planStatusFromProto(pws.Status)
	if err != nil {
		return PlanWithStatus{}, err
	}
	statusHistory := []PlanStatus{}
	statusHistory = append(statusHistory, status)
	for _, s := range pws.StatusHistory {
		ps, err := planStatusFromProto(s)
		if err != nil {
			return PlanWithStatus{}, err
		}
		statusHistory = append(statusHistory, ps)
	}

	return PlanWithStatus{
		Plan:          plan,
		StatusHistory: statusHistory,
	}, nil
}

// planStatusFromProto converts a *pb.PlanStatus to a PlanStatus.
func planStatusFromProto(ps *pb.PlanStatus) (PlanStatus, error) {
	if ps == nil {
		return PlanStatus{}, errors.New("received nil *pb.PlanStatus")
	}

	return PlanStatus{
		State:     planStateFromProto(ps.State),
		Reason:    ps.Reason,
		Timestamp: ps.Timestamp.AsTime(),
	}, nil
}

// planStatusWithIDFromProto converts a *pb.PlanStatus to a PlanStatus.
func planStatusWithIDFromProto(ps *pb.PlanStatusWithID) (PlanStatusWithID, error) {
	if ps == nil {
		return PlanStatusWithID{}, errors.New("received nil *pb.PlanStatusWithID")
	}

	planID, err := uuid.Parse(ps.PlanId)
	if err != nil {
		return PlanStatusWithID{}, err
	}

	executionID, err := uuid.Parse(ps.ExecutionId)
	if err != nil {
		return PlanStatusWithID{}, err
	}

	status, err := planStatusFromProto(ps.Status)
	if err != nil {
		return PlanStatusWithID{}, err
	}

	if ps.ComponentName == nil {
		return PlanStatusWithID{}, errors.New("received nil *commonpb.ResourceName")
	}

	return PlanStatusWithID{
		PlanID:        planID,
		ComponentName: rprotoutils.ResourceNameFromProto(ps.ComponentName),
		ExecutionID:   executionID,
		Status:        status,
	}, nil
}

// planFromProto converts a *pb.Plan to a Plan.
func planFromProto(p *pb.Plan) (Plan, error) {
	if p == nil {
		return Plan{}, errors.New("received nil *pb.Plan")
	}

	id, err := uuid.Parse(p.Id)
	if err != nil {
		return Plan{}, err
	}

	executionID, err := uuid.Parse(p.ExecutionId)
	if err != nil {
		return Plan{}, err
	}

	if p.ComponentName == nil {
		return Plan{}, errors.New("received nil *pb.ResourceName")
	}

	plan := Plan{
		ID:            id,
		ComponentName: rprotoutils.ResourceNameFromProto(p.ComponentName),
		ExecutionID:   executionID,
	}

	if len(p.Steps) == 0 {
		return plan, nil
	}

	steps := []PlanStep{}
	for _, s := range p.Steps {
		step, err := planStepFromProto(s)
		if err != nil {
			return Plan{}, err
		}
		steps = append(steps, step)
	}

	plan.Steps = steps

	return plan, nil
}

// planStepFromProto converts a *pb.PlanStep to a PlanStep.
func planStepFromProto(s *pb.PlanStep) (PlanStep, error) {
	if s == nil {
		return PlanStep{}, errors.New("received nil *pb.PlanStep")
	}

	step := make(PlanStep)
	for k, v := range s.Step {
		name, err := resource.NewFromString(k)
		if err != nil {
			return PlanStep{}, err
		}
		step[name] = spatialmath.NewPoseFromProtobuf(v.Pose)
	}
	return step, nil
}

// planStateFromProto converts a pb.PlanState to a PlanState.
func planStateFromProto(ps pb.PlanState) PlanState {
	switch ps {
	case pb.PlanState_PLAN_STATE_IN_PROGRESS:
		return PlanStateInProgress
	case pb.PlanState_PLAN_STATE_STOPPED:
		return PlanStateStopped
	case pb.PlanState_PLAN_STATE_SUCCEEDED:
		return PlanStateSucceeded
	case pb.PlanState_PLAN_STATE_FAILED:
		return PlanStateFailed
	case pb.PlanState_PLAN_STATE_UNSPECIFIED:
		return PlanStateUnspecified
	default:
		return PlanStateUnspecified
	}
}

//nolint:dupl
func moveOnGlobeNewRequestFromProto(req *pb.MoveOnGlobeNewRequest) (MoveOnGlobeReq, error) {
	if req.Destination == nil {
		return MoveOnGlobeReq{}, errors.New("must provide a destination")
	}

	// Optionals
	heading := math.NaN()
	if req.Heading != nil {
		heading = req.GetHeading()
	}
	obstaclesProto := req.GetObstacles()
	obstacles := make([]*spatialmath.GeoObstacle, 0, len(obstaclesProto))
	for _, eachProtoObst := range obstaclesProto {
		convObst, err := spatialmath.GeoObstacleFromProtobuf(eachProtoObst)
		if err != nil {
			return MoveOnGlobeReq{}, err
		}
		obstacles = append(obstacles, convObst)
	}

	componentName := rprotoutils.ResourceNameFromProto(req.GetComponentName())
	destination := geo.NewPoint(req.GetDestination().GetLatitude(), req.GetDestination().GetLongitude())
	movementSensorName := rprotoutils.ResourceNameFromProto(req.GetMovementSensorName())
	motionCfg := configurationFromProto(req.MotionConfiguration)

	return MoveOnGlobeReq{
		ComponentName:      componentName,
		Destination:        destination,
		Heading:            heading,
		MovementSensorName: movementSensorName,
		Obstacles:          obstacles,
		MotionCfg:          motionCfg,
		Extra:              req.Extra.AsMap(),
	}, nil
}

//nolint:dupl
func moveOnGlobeRequestFromProto(req *pb.MoveOnGlobeRequest) (MoveOnGlobeReq, error) {
	if req.Destination == nil {
		return MoveOnGlobeReq{}, errors.New("must provide a destination")
	}

	// Optionals
	heading := math.NaN()
	if req.Heading != nil {
		heading = req.GetHeading()
	}
	obstaclesProto := req.GetObstacles()
	obstacles := make([]*spatialmath.GeoObstacle, 0, len(obstaclesProto))
	for _, eachProtoObst := range obstaclesProto {
		convObst, err := spatialmath.GeoObstacleFromProtobuf(eachProtoObst)
		if err != nil {
			return MoveOnGlobeReq{}, err
		}
		obstacles = append(obstacles, convObst)
	}

	componentName := rprotoutils.ResourceNameFromProto(req.GetComponentName())
	destination := geo.NewPoint(req.GetDestination().GetLatitude(), req.GetDestination().GetLongitude())
	movementSensorName := rprotoutils.ResourceNameFromProto(req.GetMovementSensorName())
	motionCfg := configurationFromProto(req.MotionConfiguration)

	return MoveOnGlobeReq{
		ComponentName:      componentName,
		Destination:        destination,
		Heading:            heading,
		MovementSensorName: movementSensorName,
		Obstacles:          obstacles,
		MotionCfg:          motionCfg,
		Extra:              req.Extra.AsMap(),
	}, nil
}
