package motion

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"path"
	"time"

	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

// Localizer is an interface which both slam and movementsensor can satisfy when wrapped respectively.
type Localizer interface {
	CurrentPosition(context.Context) (*referenceframe.PoseInFrame, error)
}

// slamLocalizer is a struct which only wraps an existing slam service.
type slamLocalizer struct {
	slam.Service
}

// NewSLAMLocalizer creates a new Localizer that relies on a slam service to report Pose.
func NewSLAMLocalizer(slam slam.Service) Localizer {
	return &slamLocalizer{Service: slam}
}

// CurrentPosition returns slam's current position.
func (s *slamLocalizer) CurrentPosition(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	pose, _, err := s.Position(ctx)
	if err != nil {
		return nil, err
	}
	return referenceframe.NewPoseInFrame(referenceframe.World, pose), err
}

// movementSensorLocalizer is a struct which only wraps an existing movementsensor.
type movementSensorLocalizer struct {
	movementsensor.MovementSensor
	origin      *geo.Point
	calibration spatialmath.Pose
}

// NewMovementSensorLocalizer creates a Localizer from a MovementSensor.
// An origin point must be specified and the localizer will return Poses relative to this point.
// A calibration pose can also be specified, which will adjust the location after it is calculated relative to the origin.
func NewMovementSensorLocalizer(ms movementsensor.MovementSensor, origin *geo.Point, calibration spatialmath.Pose) Localizer {
	return &movementSensorLocalizer{MovementSensor: ms, origin: origin, calibration: calibration}
}

type GeoPoint struct {
	Lat float64
	Lng float64
}

type GP struct {
	GeoPoint    GeoPoint
	Type        string
	StartedAt   time.Time
	CompletedAt time.Time
}

type Prop struct {
	Properties  movementsensor.Properties
	Type        string
	StartedAt   time.Time
	CompletedAt time.Time
}

type P struct {
	Pose        spatialmath.Pose
	Type        string
	StartedAt   time.Time
	CompletedAt time.Time
}

type CH struct {
	CompassHeading float64
	Type           string
	StartedAt      time.Time
	CompletedAt    time.Time
}

type O struct {
	Orientation spatialmath.Orientation
	Type        string
	StartedAt   time.Time
	CompletedAt time.Time
}

// ValidateGeopoint validates that a geopoint can be used for
// motion planning.
func ValidateGeopoint(gp *geo.Point) error {
	if math.IsNaN(gp.Lat()) {
		return errors.New("lat can't be NaN")
	}

	if math.IsNaN(gp.Lng()) {
		return errors.New("lng can't be NaN")
	}

	return nil
}

// ValidatePose validates that a pose can be used for
// motion planning.
func ValidatePose(p spatialmath.Pose) error {
	if math.IsNaN(p.Point().X) {
		return errors.New("X can't be NaN")
	}

	if math.IsNaN(p.Point().Y) {
		return errors.New("Y can't be NaN")
	}

	if math.IsNaN(p.Point().Z) {
		return errors.New("Z can't be NaN")
	}
	if math.IsNaN(p.Orientation().Quaternion().Imag) {
		return errors.New("Imag can't be NaN")
	}
	if math.IsNaN(p.Orientation().Quaternion().Jmag) {
		return errors.New("Jmag can't be NaN")
	}

	if math.IsNaN(p.Orientation().Quaternion().Kmag) {
		return errors.New("Kmag can't be NaN")
	}

	if math.IsNaN(p.Orientation().Quaternion().Real) {
		return errors.New("Real can't be NaN")
	}

	return nil
}

func LogGP(gp geo.Point, before, after time.Time) error {
	b, err := json.Marshal(GP{GeoPoint: GeoPoint{Lat: gp.Lat(), Lng: gp.Lng()}, Type: "geopoint", StartedAt: before, CompletedAt: after})
	if err != nil {
		return err
	}

	return recordSensorJson(b)
}

func LogP(p spatialmath.Pose, before, after time.Time) error {
	b, err := json.Marshal(P{Pose: p, Type: "pose", StartedAt: before, CompletedAt: after})
	if err != nil {
		return err
	}
	return recordSensorJson(b)
}

func logProp(p movementsensor.Properties, before, after time.Time) error {
	b, err := json.Marshal(Prop{Properties: p, Type: "properties", StartedAt: before, CompletedAt: after})
	if err != nil {
		return err
	}
	return recordSensorJson(b)
}

func logCH(headingLeft float64, before, after time.Time) error {
	b, err := json.Marshal(CH{CompassHeading: headingLeft, Type: "compass_heading", StartedAt: before, CompletedAt: after})
	if err != nil {
		return err
	}
	return recordSensorJson(b)
}

func logO(o spatialmath.Orientation, before, after time.Time) error {
	b, err := json.Marshal(O{Orientation: o, Type: "orientation", StartedAt: before, CompletedAt: after})
	if err != nil {
		return err
	}
	return recordSensorJson(b)
}

func recordSensorJson(b []byte) error {
	homedir, err := os.UserHomeDir()
	if err != nil {
		return err
	}

	f, err := os.OpenFile(path.Join(homedir, "localizer.jsonl"), os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0o644)
	if err != nil {
		return err
	}
	defer f.Close()
	if _, err := f.Write(append(b, '\n')); err != nil {
		return err
	}
	return nil
}

// CurrentPosition returns a movementsensor's current position.
func (m *movementSensorLocalizer) CurrentPosition(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	before := time.Now()
	gp, _, err := m.Position(ctx, nil)
	after := time.Now()
	if err != nil {
		return nil, err
	}

	if err := LogGP(*gp, before, after); err != nil {
		return nil, err
	}

	if err := ValidateGeopoint(gp); err != nil {
		return nil, err
	}
	var o spatialmath.Orientation
	before = time.Now()
	properties, err := m.Properties(ctx, nil)
	after = time.Now()
	if err != nil {
		return nil, err
	}

	if err := logProp(*properties, before, after); err != nil {
		return nil, err
	}

	switch {
	case properties.CompassHeadingSupported:
		before = time.Now()
		headingLeft, err := m.CompassHeading(ctx, nil)
		after = time.Now()
		if err != nil {
			return nil, err
		}
		if err := logCH(headingLeft, before, after); err != nil {
			return nil, err
		}

		if math.IsNaN(headingLeft) {
			return nil, errors.New("heading can't be NaN")
		}
		// CompassHeading is a left-handed value. Convert to be right-handed. Use math.Mod to ensure that 0 reports 0 rather than 360.
		theta := rdkutils.SwapCompasHeadingHandedness(headingLeft)
		o = &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: theta}
	case properties.OrientationSupported:
		before := time.Now()
		o, err = m.Orientation(ctx, nil)
		after := time.Now()
		if err != nil {
			return nil, err
		}
		if err := logO(o, before, after); err != nil {
			return nil, err
		}
	default:
		return nil, errors.New("could not get orientation from Localizer")
	}

	pose := spatialmath.NewPose(spatialmath.GeoPointToPose(gp, m.origin).Point(), o)
	return referenceframe.NewPoseInFrame(m.Name().Name, spatialmath.Compose(pose, m.calibration)), nil
}
