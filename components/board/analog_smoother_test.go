package board

import (
	"context"
	"math/rand"
	"sync"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/test"
	"go.viam.com/utils"
	"go.viam.com/utils/testutils"
)

type testReader struct {
	mu   sync.Mutex
	r    *rand.Rand
	n    int64
	lim  int64
	stop bool
}

func (t *testReader) Read(ctx context.Context, extra map[string]interface{}) (int, error) {
	t.mu.Lock()
	defer t.mu.Unlock()
	if t.stop || t.n >= t.lim {
		return 0, errStopReading
	}
	t.n++
	return t.r.Intn(100), nil
}

func TestAnalogSmoother1(t *testing.T) {
	testReader := testReader{
		r:   rand.New(rand.NewSource(11)),
		lim: 200,
	}
	defer func() {
		testReader.mu.Lock()
		defer testReader.mu.Unlock()
		testReader.stop = true
	}()

	logger := golog.NewTestLogger(t)
	tmp := SmoothAnalogReader(&testReader, AnalogConfig{}, logger)
	_, ok := tmp.(*AnalogSmoother)
	test.That(t, ok, test.ShouldBeFalse)

	as := SmoothAnalogReader(&testReader, AnalogConfig{
		AverageOverMillis: 10,
		SamplesPerSecond:  10000,
	}, logger)
	_, ok = as.(*AnalogSmoother)
	test.That(t, ok, test.ShouldBeTrue)

	testutils.WaitForAssertionWithSleep(t, 10*time.Millisecond, 200, func(tb testing.TB) {
		tb.Helper()
		v, err := as.Read(context.Background(), nil)
		test.That(tb, err, test.ShouldEqual, errStopReading)
		test.That(tb, v, test.ShouldEqual, 52)

		// need lock to access testReader.n
		testReader.mu.Lock()
		defer testReader.mu.Unlock()
		test.That(tb, testReader.n, test.ShouldEqual, testReader.lim)
	})

	err := utils.TryClose(context.Background(), as)
	test.That(t, err, test.ShouldBeNil)
}
