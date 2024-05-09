// Package videosource implements various camera models including webcam
package videosource

import (
	"context"
	_ "embed"
	"fmt"

	"github.com/pkg/errors"
	pb "go.viam.com/api/component/camera/v1"

	"go.viam.com/rdk/components/camera"
	debugLogger "go.viam.com/rdk/components/camera/videosource/logging"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
)

// ModelWebcam is the name of the webcam component.
var ModelWebcam = resource.DefaultModelFamily.WithModel("webcam")

//go:embed data/intrinsics.json
var intrinsics []byte

var data map[string]transform.PinholeCameraIntrinsics

// CameraConfig is collection of configuration options for a camera.
type CameraConfig struct {
	Label string
}

// Discover webcam attributes.
func webcamsToMap(webcams []*pb.Webcam) debugLogger.InfoMap {
	info := make(debugLogger.InfoMap)
	for _, w := range webcams {
		k := w.Name
		v := fmt.Sprintf("ID: %s\n", w.Id)
		v += fmt.Sprintf("Status: %s\n", w.Status)
		v += fmt.Sprintf("Label: %s\n", w.Label)
		v += "Properties:"
		for _, p := range w.Properties {
			v += fmt.Sprintf(" :%s=%-4d | %s=%-4d | %s=%-5s | %s=%-4.2f\n",
				"width_px", p.GetWidthPx(),
				"height_px", p.GetHeightPx(),
				"frame_format", p.GetFrameFormat(),
				"frame_rate", p.GetFrameRate(),
			)
		}
		info[k] = v
	}
	return info
}

// WebcamConfig is the attribute struct for webcams.
type WebcamConfig struct {
	CameraParameters     *transform.PinholeCameraIntrinsics `json:"intrinsic_parameters,omitempty"`
	DistortionParameters *transform.BrownConrady            `json:"distortion_parameters,omitempty"`
	Debug                bool                               `json:"debug,omitempty"`
	Format               string                             `json:"format,omitempty"`
	Path                 string                             `json:"video_path"`
	Width                int                                `json:"width_px,omitempty"`
	Height               int                                `json:"height_px,omitempty"`
	FrameRate            float32                            `json:"frame_rate,omitempty"`
}

// Validate ensures all parts of the config are valid.
func (c WebcamConfig) Validate(path string) ([]string, error) {
	if c.Width < 0 || c.Height < 0 {
		return nil, fmt.Errorf(
			"got illegal negative dimensions for width_px and height_px (%d, %d) fields set for webcam camera",
			c.Height, c.Width)
	}

	return []string{}, nil
}

// getLabelFromVideoSource returns the path from the camera or an empty string if a path is not found.
// NewWebcam returns a new source based on a webcam discovered from the given config.
func NewWebcam(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (camera.Camera, error) {
	cam := &monitoredWebcam{
		Named: conf.ResourceName().AsNamed(),
	}

	return cam, nil
}

type noopCloser struct {
	gostream.VideoSource
}

func (n *noopCloser) Close(ctx context.Context) error {
	return nil
}

func (c *monitoredWebcam) Reconfigure(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
) error {
	return nil
}

// monitoredWebcam tries to ensure its underlying camera stays connected.
type monitoredWebcam struct {
	resource.Named
}

func (c *monitoredWebcam) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, errors.New("unimplemented")
}

func (c *monitoredWebcam) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, errors.New("unimplemented")
}

func (c *monitoredWebcam) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return nil, errors.New("unimplemented")
}

func (c *monitoredWebcam) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	return nil, errors.New("unimplemented")
}

func (c *monitoredWebcam) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{}, nil
}

func (c *monitoredWebcam) Close(ctx context.Context) error {
	return nil
}
