package vision_test

import (
	"bytes"
	"context"
	"encoding/base64"
	"image"
	"io"
	"strconv"
	"testing"
	"time"

	"github.com/benbjohnson/clock"
	datasyncpb "go.viam.com/api/app/datasync/v1"
	pb "go.viam.com/api/service/vision/v1"
	"go.viam.com/test"
	"go.viam.com/utils/protoutils"
	"google.golang.org/protobuf/reflect/protoreflect"
	"google.golang.org/protobuf/types/known/anypb"
	"google.golang.org/protobuf/types/known/structpb"
	"google.golang.org/protobuf/types/known/wrapperspb"

	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/rimage"
	visionservice "go.viam.com/rdk/services/vision"
	tu "go.viam.com/rdk/testutils"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils"
	"go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	"go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
)

//nolint:lll
var viamLogoJpegB64 = []byte("/9j/4QD4RXhpZgAATU0AKgAAAAgABwESAAMAAAABAAEAAAEaAAUAAAABAAAAYgEbAAUAAAABAAAAagEoAAMAAAABAAIAAAExAAIAAAAhAAAAcgITAAMAAAABAAEAAIdpAAQAAAABAAAAlAAAAAAAAABIAAAAAQAAAEgAAAABQWRvYmUgUGhvdG9zaG9wIDIzLjQgKE1hY2ludG9zaCkAAAAHkAAABwAAAAQwMjIxkQEABwAAAAQBAgMAoAAABwAAAAQwMTAwoAEAAwAAAAEAAQAAoAIABAAAAAEAAAAgoAMABAAAAAEAAAAgpAYAAwAAAAEAAAAAAAAAAAAA/9sAhAAcHBwcHBwwHBwwRDAwMERcRERERFx0XFxcXFx0jHR0dHR0dIyMjIyMjIyMqKioqKioxMTExMTc3Nzc3Nzc3NzcASIkJDg0OGA0NGDmnICc5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ubm5ub/3QAEAAL/wAARCAAgACADASIAAhEBAxEB/8QBogAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoLEAACAQMDAgQDBQUEBAAAAX0BAgMABBEFEiExQQYTUWEHInEUMoGRoQgjQrHBFVLR8CQzYnKCCQoWFxgZGiUmJygpKjQ1Njc4OTpDREVGR0hJSlNUVVZXWFlaY2RlZmdoaWpzdHV2d3h5eoOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4eLj5OXm5+jp6vHy8/T19vf4+foBAAMBAQEBAQEBAQEAAAAAAAABAgMEBQYHCAkKCxEAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDm6K0dNu1tZsSgGNuDx0961NX09WT7ZbgcD5gPT1oA5qiul0fT1VPtlwByPlB7D1rL1K7W5mxEAI04GBjPvQB//9Dm66TRr/I+xTf8A/wrm6ASpBXgjpQB0ms34UfYof8AgWP5VzdBJY5PJNFAH//Z")

type extraFields struct {
	Height   int
	Width    int
	MimeType string
}

type fakeDetection struct {
	boundingBox *image.Rectangle
	score       float64
	label       string
}

type fakeClassification struct {
	score float64
	label string
}

const (
	serviceName     = "vision"
	captureInterval = time.Millisecond
)

var fakeDetections = []objectdetection.Detection{
	&fakeDetection{
		boundingBox: &image.Rectangle{
			Min: image.Point{X: 10, Y: 20},
			Max: image.Point{X: 110, Y: 120},
		},
		score: 0.95,
		label: "cat",
	},
}

var fakeDetections2 = []objectdetection.Detection{
	&fakeDetection{
		boundingBox: &image.Rectangle{
			Min: image.Point{X: 10, Y: 20},
			Max: image.Point{X: 110, Y: 120},
		},
		score: 0.3,
		label: "cat",
	},
}

var fakeClassifications = []classification.Classification{
	&fakeClassification{
		score: 0.85,
		label: "cat",
	},
}

var fakeClassifications2 = []classification.Classification{
	&fakeClassification{
		score: 0.49,
		label: "cat",
	},
}

var fakeObjects = []*vision.Object{}

var extra = extraFields{}

var fakeExtraFields, _ = protoutils.StructToStructPb(extra)

func (fc *fakeClassification) Score() float64 {
	return fc.score
}

func (fc *fakeClassification) Label() string {
	return fc.label
}

func (fd *fakeDetection) BoundingBox() *image.Rectangle {
	return fd.boundingBox
}

func (fd *fakeDetection) Score() float64 {
	return fd.score
}

func (fd *fakeDetection) Label() string {
	return fd.label
}

func clasToProto(classifications classification.Classifications) []*pb.Classification {
	protoCs := make([]*pb.Classification, 0, len(classifications))
	for _, c := range classifications {
		cc := &pb.Classification{
			ClassName:  c.Label(),
			Confidence: c.Score(),
		}
		protoCs = append(protoCs, cc)
	}
	return protoCs
}

func detsToProto(detections []objectdetection.Detection) []*pb.Detection {
	protoDets := make([]*pb.Detection, 0, len(detections))
	for _, det := range detections {
		box := det.BoundingBox()
		if box == nil {
			return nil
		}
		xMin := int64(box.Min.X)
		yMin := int64(box.Min.Y)
		xMax := int64(box.Max.X)
		yMax := int64(box.Max.Y)
		d := &pb.Detection{
			XMin:       &xMin,
			YMin:       &yMin,
			XMax:       &xMax,
			YMax:       &yMax,
			Confidence: det.Score(),
			ClassName:  det.Label(),
		}
		protoDets = append(protoDets, d)
	}
	return protoDets
}

func convertStringMapToAnyPBMap(params map[string]string) (map[string]*anypb.Any, error) {
	methodParams := map[string]*anypb.Any{}
	for key, paramVal := range params {
		anyVal, err := convertStringToAnyPB(paramVal)
		if err != nil {
			return nil, err
		}
		methodParams[key] = anyVal
	}
	return methodParams, nil
}

func convertStringToAnyPB(str string) (*anypb.Any, error) {
	var wrappedVal protoreflect.ProtoMessage
	if boolVal, err := strconv.ParseBool(str); err == nil {
		wrappedVal = wrapperspb.Bool(boolVal)
	} else if int64Val, err := strconv.ParseInt(str, 10, 64); err == nil {
		wrappedVal = wrapperspb.Int64(int64Val)
	} else if uint64Val, err := strconv.ParseUint(str, 10, 64); err == nil {
		wrappedVal = wrapperspb.UInt64(uint64Val)
	} else if float64Val, err := strconv.ParseFloat(str, 64); err == nil {
		wrappedVal = wrapperspb.Double(float64Val)
	} else {
		wrappedVal = wrapperspb.String(str)
	}
	anyVal, err := anypb.New(wrappedVal)
	if err != nil {
		return nil, err
	}
	return anyVal, nil
}

var methodParams, _ = convertStringMapToAnyPBMap(map[string]string{"camera_name": "camera-1", "mime_type": "image/jpeg"})

func toProto(t *testing.T, r interface{}) *structpb.Struct {
	msg, err := protoutils.StructToStructPb(r)
	test.That(t, err, test.ShouldBeNil)
	return msg
}

func TestCollectors(t *testing.T) {
	viamLogoJpeg, err := io.ReadAll(base64.NewDecoder(base64.StdEncoding, bytes.NewReader(viamLogoJpegB64)))
	test.That(t, err, test.ShouldBeNil)
	viamLogoJpegAsInts := []any{}
	for _, b := range viamLogoJpeg {
		viamLogoJpegAsInts = append(viamLogoJpegAsInts, int(b))
	}

	img := rimage.NewLazyEncodedImage(viamLogoJpeg, utils.MimeTypeJPEG)
	// 32 x 32 image
	test.That(t, img.Bounds().Dx(), test.ShouldEqual, 32)
	test.That(t, img.Bounds().Dy(), test.ShouldEqual, 32)

	expected1Struct, err := structpb.NewValue(map[string]any{
		"image": map[string]any{
			"source_name": "camera-1",
			"format":      3,
			"image":       viamLogoJpegAsInts,
		},
		"classifications": []any{
			map[string]any{
				"confidence": 0.85,
				"class_name": "cat",
			},
		},
		"detections": []any{
			map[string]any{
				"confidence": 0.95,
				"class_name": "cat",
				"x_min":      10,
				"y_min":      20,
				"x_max":      110,
				"y_max":      120,
			},
		},
		"objects": []any{},
		"extra": map[string]any{
			"fields": map[string]any{
				"Height": map[string]any{
					"Kind": map[string]any{
						"NumberValue": 32,
					},
				},
				"Width": map[string]any{
					"Kind": map[string]any{
						"NumberValue": 32,
					},
				},
				"MimeType": map[string]any{
					"Kind": map[string]any{
						"StringValue": utils.MimeTypeJPEG,
					},
				},
			},
		}})

	test.That(t, err, test.ShouldBeNil)
	expected1 := &datasyncpb.SensorData{
		Metadata: &datasyncpb.SensorMetadata{},
		Data:     &datasyncpb.SensorData_Struct{Struct: expected1Struct.GetStructValue()},
	}

	expected2Struct, err := structpb.NewValue(map[string]any{
		"image": map[string]any{
			"source_name": "camera-1",
			"format":      3,
			"image":       viamLogoJpegAsInts,
		},
		"classifications": []any{},
		"detections":      []any{},
		"objects":         []any{},
		"extra": map[string]any{
			"fields": map[string]any{
				"Height": map[string]any{
					"Kind": map[string]any{
						"NumberValue": 32,
					},
				},
				"Width": map[string]any{
					"Kind": map[string]any{
						"NumberValue": 32,
					},
				},
				"MimeType": map[string]any{
					"Kind": map[string]any{
						"StringValue": utils.MimeTypeJPEG,
					},
				},
			},
		}})

	test.That(t, err, test.ShouldBeNil)
	expected2 := &datasyncpb.SensorData{
		Metadata: &datasyncpb.SensorMetadata{},
		Data:     &datasyncpb.SensorData_Struct{Struct: expected2Struct.GetStructValue()},
	}

	tests := []struct {
		name      string
		collector data.CollectorConstructor
		expected  *datasyncpb.SensorData
		vision    visionservice.Service
	}{
		{
			name:      "CaptureAllFromCameraCollector returns non-empty CaptureAllFromCameraResp",
			collector: visionservice.NewCaptureAllFromCameraCollector,
			expected:  expected1,
			vision:    newVisionService(img),
		},
		{
			name:      "CaptureAllFromCameraCollector w/ Classifications & Detections < 0.5 returns empty CaptureAllFromCameraResp",
			collector: visionservice.NewCaptureAllFromCameraCollector,
			expected:  expected2,
			vision:    newVisionService2(img),
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			start := time.Now()
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			buf := tu.NewMockBuffer(ctx)
			params := data.CollectorParams{
				DataType:      data.CaptureTypeBinary,
				ComponentName: serviceName,
				Interval:      captureInterval,
				Logger:        logging.NewTestLogger(t),
				Clock:         clock.New(),
				Target:        buf,
				MethodParams:  methodParams,
			}

			col, err := tc.collector(tc.vision, params)
			test.That(t, err, test.ShouldBeNil)

			defer col.Close()
			col.Collect()

			tu.CheckMockBufferWrites(t, ctx, start, buf.Writes, tc.expected)
		})
	}
}

func newVisionService(img image.Image) visionservice.Service {
	v := &inject.VisionService{}
	v.CaptureAllFromCameraFunc = func(ctx context.Context, cameraName string, opts viscapture.CaptureOptions,
		extra map[string]interface{},
	) (viscapture.VisCapture, error) {
		return viscapture.VisCapture{
			Image:           img,
			Detections:      fakeDetections,
			Classifications: fakeClassifications,
		}, nil
	}

	return v
}

func newVisionService2(img image.Image) visionservice.Service {
	v := &inject.VisionService{}
	v.CaptureAllFromCameraFunc = func(ctx context.Context, cameraName string, opts viscapture.CaptureOptions,
		extra map[string]interface{},
	) (viscapture.VisCapture, error) {
		return viscapture.VisCapture{
			Image:           img,
			Detections:      fakeDetections2,
			Classifications: fakeClassifications2,
		}, nil
	}

	return v
}
