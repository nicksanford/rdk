package pointcloud

import (
	"image/color"
	"math"
	"sync"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"
)

// FlatType is the type identifier for the flat pointcloud implementation.
const FlatType = "flat"

var flatConfig = TypeConfig{
	StructureType: FlatType,
	NewWithParams: NewFlatPointCloud,
}

func init() {
	Register(flatConfig)
}

// Flag bits for flatPoint.flags.
const (
	flatFlagHasColor     uint8 = 1 << 0
	flatFlagHasValue     uint8 = 1 << 1
	flatFlagHasIntensity uint8 = 1 << 2
)

// flatPoint is a 40-byte, pointer-free struct that directly implements the Data
// interface. Because *flatPoint is a pointer type, boxing it into a Data interface
// stores only the pointer in the interface's data word — zero additional allocation.
// The Go GC does not scan pointer-free slice contents, so []flatPoint has O(1) GC
// scan cost regardless of length.
//
// Field names for the stored values use a "flat" prefix to avoid a name collision
// with the Value() and Intensity() interface methods.
type flatPoint struct {
	X, Y, Z      float64 // offset  0 — 24 bytes
	flatValue     int32   // offset 24 —  4 bytes
	flatIntensity uint16  // offset 28 —  2 bytes
	R, G, B, A   uint8   // offset 30 —  4 bytes
	flags         uint8   // offset 34 —  1 byte
	_             [5]byte // offset 35 —  5 bytes padding → total 40 bytes
}

// Data interface implementation on *flatPoint.

func (fp *flatPoint) HasColor() bool { return fp.flags&flatFlagHasColor != 0 }
func (fp *flatPoint) HasValue() bool { return fp.flags&flatFlagHasValue != 0 }

func (fp *flatPoint) RGB255() (uint8, uint8, uint8) {
	return fp.R, fp.G, fp.B
}

func (fp *flatPoint) Color() color.Color {
	return color.NRGBA{R: fp.R, G: fp.G, B: fp.B, A: fp.A}
}

func (fp *flatPoint) SetColor(c color.NRGBA) Data {
	fp.R = c.R
	fp.G = c.G
	fp.B = c.B
	fp.A = c.A
	fp.flags |= flatFlagHasColor
	return fp
}

func (fp *flatPoint) Value() int { return int(fp.flatValue) }

func (fp *flatPoint) SetValue(v int) Data {
	fp.flatValue = int32(v)
	fp.flags |= flatFlagHasValue
	return fp
}

func (fp *flatPoint) Intensity() uint16 { return fp.flatIntensity }

func (fp *flatPoint) SetIntensity(v uint16) Data {
	fp.flatIntensity = v
	fp.flags |= flatFlagHasIntensity
	return fp
}

// hashKey computes a FNV-1a hash of three float64 coordinate values.
func hashKey(x, y, z float64) uint64 {
	h := uint64(14695981039346656037)
	h ^= math.Float64bits(x)
	h *= 1099511628211
	h ^= math.Float64bits(y)
	h *= 1099511628211
	h ^= math.Float64bits(z)
	h *= 1099511628211
	return h
}

// writeDataToFlatPoint copies the fields from a Data interface into a flatPoint.
func writeDataToFlatPoint(fp *flatPoint, d Data) {
	fp.flags = 0
	if d == nil {
		return
	}
	if d.HasColor() {
		r, g, b := d.RGB255()
		fp.R = r
		fp.G = g
		fp.B = b
		// Retrieve alpha via Color() — basicData stores NRGBA so cast works.
		if c, ok := d.Color().(color.NRGBA); ok {
			fp.A = c.A
		} else {
			fp.A = 255
		}
		fp.flags |= flatFlagHasColor
	}
	if d.HasValue() {
		fp.flatValue = int32(d.Value())
		fp.flags |= flatFlagHasValue
	}
	if intensity := d.Intensity(); intensity != 0 {
		fp.flatIntensity = intensity
		fp.flags |= flatFlagHasIntensity
	}
}

// flatStorage is a contiguous, pointer-free storage backend.
//
//   - points: single []flatPoint allocation; GC treats the whole slice as O(1).
//   - indexMap: map[uint64]uint32 — both key and value are scalar types, so the GC
//     skips scanning bucket contents entirely.
//   - overflow: populated only on hash collision (rare); nil in the typical case.
type flatStorage struct {
	mu       sync.RWMutex
	points   []flatPoint
	indexMap map[uint64]uint32
	overflow map[r3.Vector]uint32
}

func newFlatStorage(size int) *flatStorage {
	return &flatStorage{
		points:   make([]flatPoint, 0, size),
		indexMap: make(map[uint64]uint32, size),
	}
}

func (fs *flatStorage) Size() int {
	fs.mu.RLock()
	defer fs.mu.RUnlock()
	return len(fs.points)
}

func (fs *flatStorage) Set(v r3.Vector, d Data) error {
	fs.mu.Lock()
	defer fs.mu.Unlock()

	if v.X > maxPreciseFloat64 || v.X < minPreciseFloat64 {
		return newOutOfRangeErr("x", v.X)
	}
	if v.Y > maxPreciseFloat64 || v.Y < minPreciseFloat64 {
		return newOutOfRangeErr("y", v.Y)
	}
	if v.Z > maxPreciseFloat64 || v.Z < minPreciseFloat64 {
		return newOutOfRangeErr("z", v.Z)
	}

	key := hashKey(v.X, v.Y, v.Z)

	if idx, found := fs.indexMap[key]; found {
		fp := &fs.points[idx]
		// Verify coordinates to detect hash collisions.
		if fp.X == v.X && fp.Y == v.Y && fp.Z == v.Z {
			writeDataToFlatPoint(fp, d)
			return nil
		}
		// Hash collision — use overflow map.
		if fs.overflow == nil {
			fs.overflow = make(map[r3.Vector]uint32)
		}
		if oidx, ok := fs.overflow[v]; ok {
			writeDataToFlatPoint(&fs.points[oidx], d)
			return nil
		}
		// New point that collides with an existing hash.
		fp2 := flatPoint{X: v.X, Y: v.Y, Z: v.Z}
		writeDataToFlatPoint(&fp2, d)
		fs.points = append(fs.points, fp2)
		fs.overflow[v] = uint32(len(fs.points) - 1)
		return nil
	}

	fp := flatPoint{X: v.X, Y: v.Y, Z: v.Z}
	writeDataToFlatPoint(&fp, d)
	fs.points = append(fs.points, fp)
	fs.indexMap[key] = uint32(len(fs.points) - 1)
	return nil
}

func (fs *flatStorage) At(x, y, z float64) (Data, bool) {
	fs.mu.RLock()
	defer fs.mu.RUnlock()

	key := hashKey(x, y, z)
	if idx, found := fs.indexMap[key]; found {
		fp := &fs.points[idx]
		if fp.X == x && fp.Y == y && fp.Z == z {
			return fp, true
		}
		// Collision — check overflow.
		if fs.overflow != nil {
			if oidx, ok := fs.overflow[r3.Vector{X: x, Y: y, Z: z}]; ok {
				return &fs.points[oidx], true
			}
		}
		return nil, false
	}
	return nil, false
}

func (fs *flatStorage) Iterate(numBatches, myBatch int, fn func(p r3.Vector, d Data) bool) {
	fs.mu.RLock()
	defer fs.mu.RUnlock()

	n := len(fs.points)
	lowerBound := 0
	upperBound := n
	if numBatches > 0 {
		batchSize := (n + numBatches - 1) / numBatches
		lowerBound = myBatch * batchSize
		upperBound = (myBatch + 1) * batchSize
	}
	if upperBound > n {
		upperBound = n
	}
	for i := lowerBound; i < upperBound; i++ {
		fp := &fs.points[i]
		p := r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z}
		if !fn(p, fp) {
			return
		}
	}
}

func (fs *flatStorage) EditSupported() bool { return true }
func (fs *flatStorage) IsOrdered() bool     { return true }

// flatPointCloud is the PointCloud implementation backed by flatStorage.
type flatPointCloud struct {
	points *flatStorage
	meta   MetaData
}

// NewFlatEmpty creates an empty flat pointcloud.
func NewFlatEmpty() PointCloud {
	return NewFlatPointCloud(0)
}

// NewFlatPointCloud creates a flat pointcloud pre-allocated for size points.
func NewFlatPointCloud(size int) PointCloud {
	return &flatPointCloud{
		points: newFlatStorage(size),
		meta:   NewMetaData(),
	}
}

func (cloud *flatPointCloud) Size() int {
	return cloud.points.Size()
}

func (cloud *flatPointCloud) MetaData() MetaData {
	return cloud.meta
}

func (cloud *flatPointCloud) At(x, y, z float64) (Data, bool) {
	return cloud.points.At(x, y, z)
}

// Set validates and stores the point, then updates metadata for new points.
func (cloud *flatPointCloud) Set(p r3.Vector, d Data) error {
	_, pointExists := cloud.At(p.X, p.Y, p.Z)
	if err := cloud.points.Set(p, d); err != nil {
		return err
	}
	if !pointExists {
		cloud.meta.Merge(p, d)
	}
	return nil
}

func (cloud *flatPointCloud) Iterate(numBatches, myBatch int, fn func(p r3.Vector, d Data) bool) {
	cloud.points.Iterate(numBatches, myBatch, fn)
}

func (cloud *flatPointCloud) FinalizeAfterReading() (PointCloud, error) {
	return cloud, nil
}

func (cloud *flatPointCloud) CreateNewRecentered(offset spatialmath.Pose) PointCloud {
	return NewFlatPointCloud(cloud.Size())
}
