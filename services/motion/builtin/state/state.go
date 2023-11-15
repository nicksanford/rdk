// Package state provides apis for motion builtin plan executions
// and manages the state of those executions
package state

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"github.com/google/uuid"
	"github.com/pkg/errors"
	"go.viam.com/utils"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var (
	// ErrUnknownResource indicates that the resource is not known.
	ErrUnknownResource = errors.New("unknown resource")
	// ErrNotFound indicates the entity was not found.
	ErrNotFound = errors.New("not found")
	// ErrExecutionStopped indicates the execution was stopped.
	ErrExecutionStopped = errors.New("execution stopped")
)

// Waypoints represent the waypoints of the plan.
type Waypoints [][]referenceframe.Input

// ExecuteResp is the response from Execute
// If execution terminated for a reason that requires replanning
// Replan will be set to true
// if execution termiated due to an error Err will be non nil
// Err without Replan indicates that execution failed & replanning
// is not possible
// If Replan is falase & Err is nil then the request succeeded.
// type ExecuteResp struct {
// 	Replan bool
// 	Err    error
// }

// PlanResp is the response from Plan.
type PlanResp struct {
	Waypoints        Waypoints
	Motionplan       motionplan.Plan
	GeoPoses         []spatialmath.GeoPose
	PosesByComponent []motion.PlanStep
}

// ExecuteResp is the response from Execute.
type ExecuteResp struct {
	// If true, the Execute function didn't reach the goal & the caller should replan
	Replan bool
	// Set if Replan is true, describes why reaplanning was triggered
	ReplanReason string
}

// PlanExecutorConstructor creates a PlannerExecutor
// if ctx is cancelled then all PlannerExecutor interface
// methods must terminate & return errors
// req is the request that will be used during planning & execution
// seedPlan (nil during the first plan) is the previous plan
// if replanning has occurred
// replanCount is the number of times replanning has occurred,
// zero the first time planning occurs.
// R is a genric type which is able to be used to create a PlannerExecutor.
type PlanExecutorConstructor[R any] func(
	ctx context.Context,
	req R,
	seedPlan motionplan.Plan,
	replanCount int,
) (PlannerExecutor, error)

// PlannerExecutor implements Plan and Execute.
// TODO: Rather than relying on the context from the constructor there should be a Stop method instead.
type PlannerExecutor interface {
	Plan() (PlanResp, error)
	Execute(Waypoints) (ExecuteResp, error)
}

type componentState struct {
	executionIDHistory []motion.ExecutionID
	executionsByID     map[motion.ExecutionID]stateExecution
}

type newPlanMsg struct {
	plan       motion.Plan
	planStatus motion.PlanStatus
}

type stateUpdateMsg struct {
	componentName resource.Name
	executionID   motion.ExecutionID
	planID        motion.PlanID
	planStatus    motion.PlanStatus
}

// a stateExecution is the struct held in the state that
// holds the history of plans & plan status updates an
// execution has exprienced & the waitGroup & cancelFunc
// required to shut down an execution's goroutine.
type stateExecution struct {
	id              motion.ExecutionID
	componentName   resource.Name
	waitGroup       *sync.WaitGroup
	cancelCauseFunc context.CancelCauseFunc
	history         []motion.PlanWithStatus
}

func (e *stateExecution) stop() {
	e.cancelCauseFunc(ErrExecutionStopped)
	e.waitGroup.Wait()
}

func (cs componentState) lastExecution() stateExecution {
	return cs.executionsByID[cs.lastExecutionID()]
}

func (cs componentState) lastExecutionID() motion.ExecutionID {
	return cs.executionIDHistory[0]
}

// execution represents the state of a motion planning execution.
// it only ever exists in state.StartExecution function & the go routine created.
type execution[R any] struct {
	id                      motion.ExecutionID
	state                   *State
	waitGroup               *sync.WaitGroup
	cancelCtx               context.Context
	cancelCauseFunc         context.CancelCauseFunc
	executorCancelCtx       context.Context
	executorCancelCauseFunc context.CancelCauseFunc
	logger                  logging.Logger
	// TODO: Make this generic across MoveOnGlobe & MoveOnMap
	componentName           resource.Name
	req                     R
	planExecutorConstructor PlanExecutorConstructor[R]
}

type planWithExecutor struct {
	plan         motion.Plan
	planExecutor PlannerExecutor
	waypoints    Waypoints
	motionplan   motionplan.Plan
}

func toGeoPosePlanSteps(resp PlanResp) ([]motion.PlanStep, error) {
	if len(resp.GeoPoses) != len(resp.PosesByComponent) {
		msg := "PlanResp.GeoPoses (len: %d) & PlanResp.PosesByComponent (len: %d) must have the same length"
		return nil, fmt.Errorf(msg, len(resp.GeoPoses), len(resp.PosesByComponent))
	}
	steps := make([]motion.PlanStep, 0, len(resp.PosesByComponent))
	for i, ps := range resp.PosesByComponent {
		if len(ps) == 0 {
			continue
		}

		if l := len(ps); l > 1 {
			return nil, fmt.Errorf("only single component or fewer plan steps supported, received plan step with %d componenents", l)
		}

		var resourceName resource.Name
		for k := range ps {
			resourceName = k
		}
		geoPose := resp.GeoPoses[i]
		heading := math.Mod(math.Abs(geoPose.Heading()-360), 360)
		o := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: heading}
		poseContainingGeoPose := spatialmath.NewPose(r3.Vector{X: geoPose.Location().Lat(), Y: geoPose.Location().Lng()}, o)
		steps = append(steps, map[resource.Name]spatialmath.Pose{resourceName: poseContainingGeoPose})
	}
	return steps, nil
}

// NewPlan creates a new motion.Plan from an execution & returns an error if one was not able to be created.
func (e *execution[R]) newPlanWithExecutor(seedPlan motionplan.Plan, replanCount int) (planWithExecutor, error) {
	pe, err := e.planExecutorConstructor(e.executorCancelCtx, e.req, seedPlan, replanCount)
	if err != nil {
		return planWithExecutor{}, err
	}
	resp, err := pe.Plan()
	if err != nil {
		return planWithExecutor{}, err
	}
	// TODO: TEMP: Currently this is assuming that the Executor is implemented by MoveOnGlobe & that
	// we are going to embed the GeoPose in a Pose.
	steps, err := toGeoPosePlanSteps(resp)
	if err != nil {
		return planWithExecutor{}, err
	}
	plan := motion.Plan{
		ID:            uuid.New(),
		ExecutionID:   e.id,
		ComponentName: e.componentName,
		Steps:         steps,
	}
	return planWithExecutor{plan: plan, planExecutor: pe, waypoints: resp.Waypoints, motionplan: resp.Motionplan}, nil
}

// Start starts an execution with a given plan.
func (e *execution[R]) start() error {
	var replanCount int
	originalPlanWithExecutor, err := e.newPlanWithExecutor(nil, replanCount)
	if err != nil {
		return err
	}
	e.notifyStateNewExecution(e.toStateExecution(), originalPlanWithExecutor.plan, time.Now())
	// We need to add to both the state & execution waitgroups
	// B/c both the state & the stateExecution need to know if this
	// goroutine have termianted.
	// state.Stop() needs to wait for ALL execution goroutines to terminate before
	// returning in order to not leak.
	// Similarly stateExecution.stop(), which is called by state.StopExecutionByResource
	// needs to wait for its 1 execution go routine to termiante before returning.
	// As a result, both waitgroups need to be written to.
	e.state.waitGroup.Add(1)
	e.waitGroup.Add(1)

	utils.PanicCapturingGo(func() {
		defer e.state.waitGroup.Done()
		defer e.waitGroup.Done()

		lastPWE := originalPlanWithExecutor
		// Exit conditions of this loop:
		// 1. The execution's context was cancelled, which happens if the state's Stop() was called or
		// StopExecutionByResource was called for this resource
		// 2. the execution succeeded
		// 3. the execution failed
		// 4. replanning failed
		for {
			resChan := make(chan struct {
				resp ExecuteResp
				err  error
			}, 1)
			utils.PanicCapturingGo(func() {
				replan, err := lastPWE.planExecutor.Execute(lastPWE.waypoints)
				resChan <- struct {
					resp ExecuteResp
					err  error
				}{replan, err}
			})
			select {
			case <-e.cancelCtx.Done():
				e.notifyStatePlanStopped(lastPWE.plan, time.Now())
				e.executorCancelCauseFunc(context.Cause(e.cancelCtx))
				return
			case res := <-resChan:
				// success
				if !res.resp.Replan && res.err == nil {
					e.notifyStatePlanSucceeded(lastPWE.plan, time.Now())
					return
				}

				// failure
				if !res.resp.Replan && res.err != nil {
					e.notifyStatePlanFailed(lastPWE.plan, res.err.Error(), time.Now())
					return
				}

				// replan
				// TODO: Right now we never provide a reason, we should
				replanReason := "replan triggered without providing a reason"
				if res.err != nil {
					replanReason = res.err.Error()
				}

				replanCount++
				newPWE, err := e.newPlanWithExecutor(lastPWE.motionplan, replanCount)
				// replan failed
				if err != nil {
					msg := "failed to replan for execution %s and component: %s, " +
						"due to replan reason: %s, tried setting previous plan %s " +
						"to failed due to error: %s\n"
					e.logger.Warnf(msg, e.id, e.componentName, replanReason, lastPWE.plan.ID, err.Error())

					e.notifyStatePlanFailed(lastPWE.plan, err.Error(), time.Now())
					return
				}

				e.logger.Debugf("updating last plan %s\n", lastPWE.plan.ID)
				e.notifyStatePlanFailed(lastPWE.plan, replanReason, time.Now())
				e.logger.Debugf("updating new plan %s\n", newPWE.plan.ID.String())
				e.notifyStateNewPlan(newPWE.plan, time.Now())
				lastPWE = newPWE
			}
		}
	})

	return nil
}

func (e *execution[R]) toStateExecution() stateExecution {
	return stateExecution{
		id:              e.id,
		componentName:   e.componentName,
		waitGroup:       e.waitGroup,
		cancelCauseFunc: e.cancelCauseFunc,
	}
}

// NOTE: We hold the lock for both updateStateNewExecution & updateStateNewPlan to ensure no readers
// are able to see a state where the execution exists but does not have a plan with a status.
func (e *execution[R]) notifyStateNewExecution(execution stateExecution, plan motion.Plan, time time.Time) {
	e.state.mu.Lock()
	defer e.state.mu.Unlock()

	e.state.updateStateNewExecution(execution)
	msg := newPlanMsg{
		plan:       plan,
		planStatus: motion.PlanStatus{State: motion.PlanStateInProgress, Timestamp: time},
	}
	e.state.updateStateNewPlan(msg)
}

func (e *execution[R]) notifyStateNewPlan(plan motion.Plan, time time.Time) {
	e.state.mu.Lock()
	defer e.state.mu.Unlock()
	e.state.updateStateNewPlan(newPlanMsg{
		plan:       plan,
		planStatus: motion.PlanStatus{State: motion.PlanStateInProgress, Timestamp: time},
	})
}

func (e *execution[R]) notifyStatePlanFailed(plan motion.Plan, reason string, time time.Time) {
	e.state.mu.Lock()
	defer e.state.mu.Unlock()
	e.state.updateStateStatusUpdate(stateUpdateMsg{
		componentName: e.componentName,
		executionID:   e.id,
		planID:        plan.ID,
		planStatus:    motion.PlanStatus{State: motion.PlanStateFailed, Timestamp: time, Reason: &reason},
	})
}

func (e *execution[R]) notifyStatePlanSucceeded(plan motion.Plan, time time.Time) {
	e.state.mu.Lock()
	defer e.state.mu.Unlock()
	e.state.updateStateStatusUpdate(stateUpdateMsg{
		componentName: e.componentName,
		executionID:   e.id,
		planID:        plan.ID,
		planStatus:    motion.PlanStatus{State: motion.PlanStateSucceeded, Timestamp: time},
	})
}

func (e *execution[R]) notifyStatePlanStopped(plan motion.Plan, time time.Time) {
	e.state.mu.Lock()
	defer e.state.mu.Unlock()
	e.state.updateStateStatusUpdate(stateUpdateMsg{
		componentName: e.componentName,
		executionID:   e.id,
		planID:        plan.ID,
		planStatus:    motion.PlanStatus{State: motion.PlanStateStopped, Timestamp: time},
	})
}

// State is the state of the builtin motion service
// It keeps track of the builtin motion service's executions.
type State struct {
	waitGroup       *sync.WaitGroup
	cancelCtx       context.Context
	cancelCauseFunc context.CancelCauseFunc
	logger          logging.Logger
	// mu protects the componentStateByComponent
	mu                        sync.RWMutex
	componentStateByComponent map[resource.Name]componentState
}

// NewState creates a new state.
func NewState(ctx context.Context, logger logging.Logger) *State {
	cancelCtx, cancelFunc := context.WithCancelCause(ctx)
	s := State{
		cancelCtx:                 cancelCtx,
		cancelCauseFunc:           cancelFunc,
		waitGroup:                 &sync.WaitGroup{},
		componentStateByComponent: make(map[resource.Name]componentState),
		logger:                    logger,
	}
	return &s
}

// StartExecution creates a new execution from a state.
func StartExecution[R any](
	s *State,
	componentName resource.Name,
	req R,
	planExecutorConstructor PlanExecutorConstructor[R],
) (motion.ExecutionID, error) {
	if s == nil {
		return uuid.Nil, errors.New("state is nil")
	}

	if err := s.ValidateNoActiveExecutionID(componentName); err != nil {
		return uuid.Nil, err
	}

	// the state being cancelled should cause all executions derived from that state to also be cancelled
	cancelCtx, cancelCauseFunc := context.WithCancelCause(s.cancelCtx)
	executorCancelCtx, executorCancelCauseFunc := context.WithCancelCause(context.Background())
	e := execution[R]{
		id:                      uuid.New(),
		state:                   s,
		cancelCtx:               cancelCtx,
		cancelCauseFunc:         cancelCauseFunc,
		executorCancelCtx:       executorCancelCtx,
		executorCancelCauseFunc: executorCancelCauseFunc,
		waitGroup:               &sync.WaitGroup{},
		logger:                  s.logger,
		req:                     req,
		componentName:           componentName,
		planExecutorConstructor: planExecutorConstructor,
	}

	if err := e.start(); err != nil {
		return uuid.Nil, err
	}

	return e.id, nil
}

// Stop stops all executions within the State.
func (s *State) Stop() {
	s.cancelCauseFunc(ErrExecutionStopped)
	s.waitGroup.Wait()
}

// StopExecutionByResource stops the active execution with a given resource name in the State.
func (s *State) StopExecutionByResource(componentName resource.Name) error {
	// Read lock held to get the execution
	s.mu.RLock()
	componentExectionState, exists := s.componentStateByComponent[componentName]

	// return error if component name is not in StateMap
	if !exists {
		s.mu.RUnlock()
		return ErrUnknownResource
	}

	e, exists := componentExectionState.executionsByID[componentExectionState.lastExecutionID()]
	if !exists {
		s.mu.RUnlock()
		return ErrNotFound
	}
	s.mu.RUnlock()

	// lock released while waiting for the execution to stop as the execution stopping requires writing to the state
	// which must take a lock
	e.stop()
	return nil
}

// PlanHistory returns the plans with statuses of the resource
// By default returns all plans from the most recent execution of the resoure
// If the ExecutionID is provided, returns the plans of the ExecutionID rather
// than the most recent execution
// If LastPlanOnly is provided then only the last plan is returned for the execution
// with the ExecutionID if it is provided, or the last execution
// for that component otherwise.
func (s *State) PlanHistory(req motion.PlanHistoryReq) ([]motion.PlanWithStatus, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	cs, exists := s.componentStateByComponent[req.ComponentName]
	if !exists {
		return nil, ErrUnknownResource
	}

	executionID := req.ExecutionID

	// last plan only
	if req.LastPlanOnly {
		if ex := cs.lastExecution(); executionID == uuid.Nil || executionID == ex.id {
			history := make([]motion.PlanWithStatus, 1)
			copy(history, ex.history)
			return history, nil
		}

		// if executionID is provided & doesn't match the last execution for the component
		if ex, exists := cs.executionsByID[executionID]; exists {
			history := make([]motion.PlanWithStatus, 1)
			copy(history, ex.history)
			return history, nil
		}
		return nil, ErrNotFound
	}

	// specific execution id when lastPlanOnly is NOT enabled
	if executionID != uuid.Nil {
		if ex, exists := cs.executionsByID[executionID]; exists {
			history := make([]motion.PlanWithStatus, len(ex.history))
			copy(history, ex.history)
			return history, nil
		}
		return nil, ErrNotFound
	}

	ex := cs.lastExecution()
	history := make([]motion.PlanWithStatus, len(cs.lastExecution().history))
	copy(history, ex.history)
	return history, nil
}

// ListPlanStatuses returns the status of plans created by MoveOnGlobe requests
// that are executing OR are part of an execution which changed it state
// within the a 24HR TTL OR until the robot reinitializes.
// If OnlyActivePlans is provided, only returns plans which are in non terminal states.
func (s *State) ListPlanStatuses(req motion.ListPlanStatusesReq) ([]motion.PlanStatusWithID, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	statuses := []motion.PlanStatusWithID{}
	if req.OnlyActivePlans {
		for name := range s.componentStateByComponent {
			if e, err := s.activeExecution(name); err == nil {
				statuses = append(statuses, motion.PlanStatusWithID{
					ExecutionID:   e.id,
					ComponentName: e.componentName,
					PlanID:        e.history[0].Plan.ID,
					Status:        e.history[0].StatusHistory[0],
				})
			}
		}
		return statuses, nil
	}

	for _, cs := range s.componentStateByComponent {
		for _, executionID := range cs.executionIDHistory {
			e, exists := cs.executionsByID[executionID]
			if !exists {
				return nil, errors.New("state is corrupted")
			}
			for _, pws := range e.history {
				statuses = append(statuses, motion.PlanStatusWithID{
					ExecutionID:   e.id,
					ComponentName: e.componentName,
					PlanID:        pws.Plan.ID,
					Status:        pws.StatusHistory[0],
				})
			}
		}
	}

	return statuses, nil
}

// ValidateNoActiveExecutionID returns an error if there is already an active
// Execution for the resource name within the State.
func (s *State) ValidateNoActiveExecutionID(name resource.Name) error {
	if es, err := s.activeExecution(name); err == nil {
		return fmt.Errorf("there is already an active executionID: %s", es.id)
	}
	return nil
}

func (s *State) updateStateNewExecution(newE stateExecution) {
	cs, exists := s.componentStateByComponent[newE.componentName]

	if exists {
		_, exists = cs.executionsByID[newE.id]
		if exists {
			err := fmt.Errorf("unexpected ExecutionID already exists %s", newE.id)
			s.logger.Error(err.Error())
			return
		}
		cs.executionsByID[newE.id] = newE
		cs.executionIDHistory = append([]motion.ExecutionID{newE.id}, cs.executionIDHistory...)
		s.componentStateByComponent[newE.componentName] = cs
	} else {
		s.componentStateByComponent[newE.componentName] = componentState{
			executionIDHistory: []motion.ExecutionID{newE.id},
			executionsByID:     map[motion.ExecutionID]stateExecution{newE.id: newE},
		}
	}
}

func (s *State) updateStateNewPlan(newPlan newPlanMsg) {
	if newPlan.planStatus.State != motion.PlanStateInProgress {
		err := errors.New("handleNewPlan received a plan status other than in progress")
		s.logger.Error(err.Error())
		return
	}

	activeExecutionID := s.componentStateByComponent[newPlan.plan.ComponentName].lastExecutionID()
	if newPlan.plan.ExecutionID != activeExecutionID {
		e := "got new plan for inactive execution: active executionID %s, planID: %s, component: %s, plan executionID: %s"
		err := fmt.Errorf(e, activeExecutionID, newPlan.plan.ID, newPlan.plan.ComponentName, newPlan.plan.ExecutionID)
		s.logger.Error(err.Error())
		return
	}
	execution := s.componentStateByComponent[newPlan.plan.ComponentName].executionsByID[newPlan.plan.ExecutionID]
	pws := []motion.PlanWithStatus{{Plan: newPlan.plan, StatusHistory: []motion.PlanStatus{newPlan.planStatus}}}
	// prepend  to executions.history so that lower indices are newer
	execution.history = append(pws, execution.history...)

	s.componentStateByComponent[newPlan.plan.ComponentName].executionsByID[newPlan.plan.ExecutionID] = execution
}

func (s *State) updateStateStatusUpdate(update stateUpdateMsg) {
	switch update.planStatus.State {
	// terminal states
	case motion.PlanStateSucceeded, motion.PlanStateFailed, motion.PlanStateStopped:
	default:
		err := fmt.Errorf("unexpected PlanState %v in update %#v", update.planStatus.State, update)
		s.logger.Error(err.Error())
		return
	}
	componentExecutions, exists := s.componentStateByComponent[update.componentName]
	if !exists {
		err := errors.New("updated component doesn't exist")
		s.logger.Error(err.Error())
		return
	}
	// copy the execution
	execution := componentExecutions.executionsByID[update.executionID]
	lastPlanWithStatus := execution.history[0]
	if lastPlanWithStatus.Plan.ID != update.planID {
		err := fmt.Errorf("status update for plan %s is not for last plan: %s", update.planID, lastPlanWithStatus.Plan.ID)
		s.logger.Error(err.Error())
		return
	}
	lastPlanWithStatus.StatusHistory = append([]motion.PlanStatus{update.planStatus}, lastPlanWithStatus.StatusHistory...)
	// write updated last plan back to history
	execution.history[0] = lastPlanWithStatus
	// write the execution with the new history to the component execution state copy
	componentExecutions.executionsByID[update.executionID] = execution
	// write the component execution state copy back to the state
	s.componentStateByComponent[update.componentName] = componentExecutions
}

func (s *State) activeExecution(name resource.Name) (stateExecution, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	if cs, exists := s.componentStateByComponent[name]; exists {
		es := cs.lastExecution()

		if _, exists := motion.TerminalStateSet[es.history[0].StatusHistory[0].State]; exists {
			return stateExecution{}, ErrNotFound
		}
		return es, nil
	}
	return stateExecution{}, ErrUnknownResource
}
