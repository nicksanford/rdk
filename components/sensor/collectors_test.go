package sensor_test

import (
	"context"
	"testing"
	"time"

	"github.com/benbjohnson/clock"
	datasyncpb "go.viam.com/api/app/datasync/v1"
	"go.viam.com/test"
	"google.golang.org/protobuf/types/known/structpb"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	tu "go.viam.com/rdk/testutils"
	"go.viam.com/rdk/testutils/inject"
)

const (
	captureInterval = time.Millisecond
)

var readingMap = map[string]any{"reading1": false, "reading2": "test"}

func TestCollectors(t *testing.T) {
	start := time.Now()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	buf := tu.NewMockBuffer(ctx)
	params := data.CollectorParams{
		DataType:      data.CaptureTypeTabular,
		ComponentName: "sensor",
		Interval:      captureInterval,
		Logger:        logging.NewTestLogger(t),
		Target:        buf,
		Clock:         clock.New(),
	}

	sens := newSensor()
	col, err := sensor.NewReadingsCollector(sens, params)
	test.That(t, err, test.ShouldBeNil)

	defer col.Close()
	col.Collect()

	tu.CheckMockBufferWrites(t, ctx, start, buf.Writes, &datasyncpb.SensorData{
		Metadata: &datasyncpb.SensorMetadata{},
		Data: &datasyncpb.SensorData_Struct{Struct: &structpb.Struct{
			Fields: map[string]*structpb.Value{
				"readings": structpb.NewStructValue(&structpb.Struct{
					Fields: map[string]*structpb.Value{
						"reading1": structpb.NewBoolValue(false),
						"reading2": structpb.NewStringValue("test"),
					},
				}),
			},
		}},
	})
}

func newSensor() sensor.Sensor {
	s := &inject.Sensor{}
	s.ReadingsFunc = func(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
		return readingMap, nil
	}
	return s
}
