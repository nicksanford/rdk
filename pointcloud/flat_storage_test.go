package pointcloud

import (
	"image/color"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"
)

func TestFlatStorage(t *testing.T) {
	fs := newFlatStorage(0)
	test.That(t, fs.IsOrdered(), test.ShouldEqual, true)
	testPointCloudStorage(t, fs)
}

func BenchmarkFlatStorage(b *testing.B) {
	fs := newFlatStorage(0)
	benchPointCloudStorage(b, fs)
}

func TestFlatDataFlags(t *testing.T) {
	fp := &flatPoint{}

	// No flags set initially.
	test.That(t, fp.HasColor(), test.ShouldBeFalse)
	test.That(t, fp.HasValue(), test.ShouldBeFalse)
	test.That(t, fp.Intensity(), test.ShouldEqual, uint16(0))

	// Set color.
	fp.SetColor(color.NRGBA{R: 1, G: 2, B: 3, A: 255})
	test.That(t, fp.HasColor(), test.ShouldBeTrue)
	r, g, b := fp.RGB255()
	test.That(t, r, test.ShouldEqual, uint8(1))
	test.That(t, g, test.ShouldEqual, uint8(2))
	test.That(t, b, test.ShouldEqual, uint8(3))

	// Set value.
	fp.SetValue(42)
	test.That(t, fp.HasValue(), test.ShouldBeTrue)
	test.That(t, fp.Value(), test.ShouldEqual, 42)

	// Negative value.
	fp.SetValue(-7)
	test.That(t, fp.Value(), test.ShouldEqual, -7)

	// Set intensity.
	fp.SetIntensity(1000)
	test.That(t, fp.Intensity(), test.ShouldEqual, uint16(1000))

	// All three flags should be set.
	test.That(t, fp.flags, test.ShouldEqual, flatFlagHasColor|flatFlagHasValue|flatFlagHasIntensity)
}

func TestFlatDataMutation(t *testing.T) {
	fs := newFlatStorage(1)

	// Insert a colored point.
	err := fs.Set(r3.Vector{X: 1, Y: 2, Z: 3}, NewColoredData(color.NRGBA{R: 10, G: 20, B: 30, A: 255}))
	test.That(t, err, test.ShouldBeNil)

	d, found := fs.At(1, 2, 3)
	test.That(t, found, test.ShouldBeTrue)

	// Mutation via SetColor should update the underlying flatPoint in the slice.
	d.SetColor(color.NRGBA{R: 50, G: 60, B: 70, A: 255})

	// Re-fetch; should see updated values (Data is a *flatPoint into the slice).
	d2, _ := fs.At(1, 2, 3)
	r2, g2, b2 := d2.RGB255()
	test.That(t, r2, test.ShouldEqual, uint8(50))
	test.That(t, g2, test.ShouldEqual, uint8(60))
	test.That(t, b2, test.ShouldEqual, uint8(70))
}

func TestFlatHashCollision(t *testing.T) {
	fs := newFlatStorage(4)

	pA := r3.Vector{X: 1, Y: 2, Z: 3}
	pB := r3.Vector{X: 4, Y: 5, Z: 6}
	keyB := hashKey(pB.X, pB.Y, pB.Z)

	err := fs.Set(pA, NewColoredData(color.NRGBA{R: 1}))
	test.That(t, err, test.ShouldBeNil)

	// Inject collision: make keyB point to the entry for pA (index 0).
	fs.indexMap[keyB] = 0

	err = fs.Set(pB, NewColoredData(color.NRGBA{R: 2}))
	test.That(t, err, test.ShouldBeNil)

	// Both points should be retrievable.
	dA, foundA := fs.At(pA.X, pA.Y, pA.Z)
	test.That(t, foundA, test.ShouldBeTrue)
	rA, _, _ := dA.RGB255()
	test.That(t, rA, test.ShouldEqual, uint8(1))

	dB, foundB := fs.At(pB.X, pB.Y, pB.Z)
	test.That(t, foundB, test.ShouldBeTrue)
	rB, _, _ := dB.RGB255()
	test.That(t, rB, test.ShouldEqual, uint8(2))

	// Overwrite pB via overflow path.
	err = fs.Set(pB, NewColoredData(color.NRGBA{R: 99}))
	test.That(t, err, test.ShouldBeNil)
	dB2, _ := fs.At(pB.X, pB.Y, pB.Z)
	rB2, _, _ := dB2.RGB255()
	test.That(t, rB2, test.ShouldEqual, uint8(99))
}

func TestFlatNaNHandling(t *testing.T) {
	fs := newFlatStorage(0)
	// NaN is within the precise float range so no error; verify no panic.
	nan := math.NaN()
	err := fs.Set(r3.Vector{X: nan, Y: 0, Z: 0}, NewBasicData())
	test.That(t, err, test.ShouldBeNil)
}
