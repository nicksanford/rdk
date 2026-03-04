package pointcloud

import (
	"context"
	"image/color"
	"math/rand"
	"runtime"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/spatialmath"
)

var cloudSizes = []int{1_000, 10_000, 100_000, 290_000}

var implementations = []struct {
	name    string
	factory func(int) PointCloud
}{
	{"basic", NewBasicPointCloud},
	{"flat", NewFlatPointCloud},
}

// makeCloud fills a PointCloud with n distinct points using a deterministic RNG.
func makeCloud(factory func(int) PointCloud, n int) PointCloud {
	pc := factory(n)
	r := rand.New(rand.NewSource(42))
	const scale = 10_000.0
	for i := 0; i < n; i++ {
		p := r3.Vector{
			X: (r.Float64() - 0.5) * scale,
			Y: (r.Float64() - 0.5) * scale,
			Z: (r.Float64() - 0.5) * scale,
		}
		d := NewColoredData(color.NRGBA{
			R: uint8(r.Intn(256)),
			G: uint8(r.Intn(256)),
			B: uint8(r.Intn(256)),
			A: 255,
		})
		if err := pc.Set(p, d); err != nil {
			panic(err)
		}
	}
	return pc
}

// collectPoints returns the slice of all points in insertion order.
func collectPoints(pc PointCloud) []r3.Vector {
	pts := make([]r3.Vector, 0, pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, _ Data) bool {
		pts = append(pts, p)
		return true
	})
	return pts
}

// BenchmarkIterate measures full sequential scan throughput.
func BenchmarkIterate(b *testing.B) {
	for _, impl := range implementations {
		for _, n := range cloudSizes {
			pc := makeCloud(impl.factory, n)
			b.Run(impl.name+"/"+itoa(n), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					pc.Iterate(0, 0, func(_ r3.Vector, _ Data) bool { return true })
				}
			})
		}
	}
}

// BenchmarkAtRandom measures random point lookup.
func BenchmarkAtRandom(b *testing.B) {
	for _, impl := range implementations {
		for _, n := range cloudSizes {
			pc := makeCloud(impl.factory, n)
			pts := collectPoints(pc)
			r := rand.New(rand.NewSource(7))
			b.Run(impl.name+"/"+itoa(n), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					p := pts[r.Intn(len(pts))]
					_, ok := pc.At(p.X, p.Y, p.Z)
					if !ok {
						b.Fatal("point not found")
					}
				}
			})
		}
	}
}

// BenchmarkSet measures point insertion throughput.
func BenchmarkSet(b *testing.B) {
	for _, impl := range implementations {
		for _, n := range cloudSizes {
			pts := make([]r3.Vector, n)
			r := rand.New(rand.NewSource(13))
			const scale = 10_000.0
			for i := range pts {
				pts[i] = r3.Vector{
					X: (r.Float64() - 0.5) * scale,
					Y: (r.Float64() - 0.5) * scale,
					Z: (r.Float64() - 0.5) * scale,
				}
			}
			d := NewColoredData(color.NRGBA{R: 1, G: 2, B: 3, A: 255})
			b.Run(impl.name+"/"+itoa(n), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					pc := impl.factory(n)
					for _, p := range pts {
						if err := pc.Set(p, d); err != nil {
							b.Fatal(err)
						}
					}
				}
			})
		}
	}
}

// BenchmarkSetAllocsPerOp isolates the per-Set allocation cost.
func BenchmarkSetAllocsPerOp(b *testing.B) {
	d := NewColoredData(color.NRGBA{R: 1, G: 2, B: 3, A: 255})
	for _, impl := range implementations {
		b.Run(impl.name, func(b *testing.B) {
			b.ReportAllocs()
			pc := impl.factory(b.N)
			for i := 0; i < b.N; i++ {
				p := r3.Vector{X: float64(i), Y: float64(i), Z: float64(i)}
				if err := pc.Set(p, d); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

// BenchmarkAtAllocsPerOp isolates the per-At allocation cost.
func BenchmarkAtAllocsPerOp(b *testing.B) {
	n := 10_000
	for _, impl := range implementations {
		pc := makeCloud(impl.factory, n)
		pts := collectPoints(pc)
		r := rand.New(rand.NewSource(3))
		b.Run(impl.name, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				p := pts[r.Intn(len(pts))]
				_, ok := pc.At(p.X, p.Y, p.Z)
				if !ok {
					b.Fatal("point not found")
				}
			}
		})
	}
}

// BenchmarkMemoryPerPoint reports heap bytes consumed per stored point.
func BenchmarkMemoryPerPoint(b *testing.B) {
	n := 100_000
	for _, impl := range implementations {
		b.Run(impl.name, func(b *testing.B) {
			b.ReportAllocs()
			b.StopTimer()
			runtime.GC()
			var before runtime.MemStats
			runtime.ReadMemStats(&before)
			b.StartTimer()

			pc := makeCloud(impl.factory, n)
			_ = pc

			b.StopTimer()
			runtime.GC()
			var after runtime.MemStats
			runtime.ReadMemStats(&after)

			heapDelta := int64(after.HeapInuse) - int64(before.HeapInuse)
			if heapDelta < 0 {
				heapDelta = 0
			}
			b.ReportMetric(float64(heapDelta)/float64(n), "bytes/point")
		})
	}
}

// BenchmarkGCPause measures stop-the-world GC pause time for a large cloud.
func BenchmarkGCPause(b *testing.B) {
	n := 290_000
	for _, impl := range implementations {
		b.Run(impl.name, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				b.StopTimer()
				pc := makeCloud(impl.factory, n)
				_ = pc
				runtime.GC() // warm up

				var before runtime.MemStats
				runtime.ReadMemStats(&before)
				b.StartTimer()

				runtime.GC()

				b.StopTimer()
				var after runtime.MemStats
				runtime.ReadMemStats(&after)

				pauseNs := int64(after.PauseTotalNs) - int64(before.PauseTotalNs)
				if pauseNs < 0 {
					pauseNs = 0
				}
				b.ReportMetric(float64(pauseNs), "GCpause-ns")
			}
		})
	}
}

// BenchmarkMergeFlat benchmarks MergePointClouds with flat-backed clouds.
func BenchmarkMergeFlat(b *testing.B) {
	inA := makeCloud(NewFlatPointCloud, 10_000)
	inB := NewFlatPointCloud(0)
	err := ApplyOffset(inA, spatialmath.NewPoseFromPoint(r3.Vector{1000, 1000, 1000}), inB)
	test.That(b, err, test.ShouldBeNil)

	fs := []CloudAndOffsetFunc{
		func(_ context.Context) (PointCloud, spatialmath.Pose, error) {
			return inA, spatialmath.NewPoseFromPoint(r3.Vector{1, 1, 1}), nil
		},
		func(_ context.Context) (PointCloud, spatialmath.Pose, error) {
			return inB, spatialmath.NewPoseFromPoint(r3.Vector{1, 1, 1}), nil
		},
	}
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		out := NewFlatPointCloud(0)
		if err := MergePointClouds(context.Background(), fs, out); err != nil {
			b.Fatal(err)
		}
	}
}

// BenchmarkApplyOffsetFlat benchmarks ApplyOffset with flat-backed clouds.
func BenchmarkApplyOffsetFlat(b *testing.B) {
	in := makeCloud(NewFlatPointCloud, 10_000)
	transPose := spatialmath.NewPoseFromPoint(r3.Vector{0, 99, 0})
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		out := NewFlatPointCloud(0)
		if err := ApplyOffset(in, transPose, out); err != nil {
			b.Fatal(err)
		}
		if out.Size() != in.Size() {
			b.Fatalf("size mismatch: got %d want %d", out.Size(), in.Size())
		}
	}
}

// itoa is a small int-to-string helper to avoid importing strconv in the benchmark names.
func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	buf := [20]byte{}
	pos := len(buf)
	for n > 0 {
		pos--
		buf[pos] = byte('0' + n%10)
		n /= 10
	}
	return string(buf[pos:])
}
