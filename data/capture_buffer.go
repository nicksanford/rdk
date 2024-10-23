package data

import (
	"sync"

	v1 "go.viam.com/api/app/datasync/v1"
	anypb "google.golang.org/protobuf/types/known/anypb"
)

// CaptureBufferedWriter is a buffered, persistent queue of SensorData.
type CaptureBufferedWriter interface {
	Write(item CaptureResponseWithTimeStamps) error
	Flush() error
	Path() string
}

// CaptureBuffer is a persistent queue of SensorData backed by a series of *data.CaptureFile.
type CaptureBuffer struct {
	Directory          string
	MetaData           *MetaData
	nextFile           *CaptureFile
	lock               sync.Mutex
	maxCaptureFileSize int64
}

type DataType int32

const (
	DataType_DATA_TYPE_UNSPECIFIED    DataType = 0
	DataType_DATA_TYPE_BINARY_SENSOR  DataType = 1
	DataType_DATA_TYPE_TABULAR_SENSOR DataType = 2
)

type MetaData struct {
	ComponentType    string
	ComponentName    string
	MethodName       string
	Type             DataType
	MethodParameters map[string]*anypb.Any
	FileExtension    string
	Tags             []string
	BoundingBoxes    []BoundingBox
}

func (m *MetaData) ToProto() *v1.DataCaptureMetadata {
	return &v1.DataCaptureMetadata{
		ComponentType:    m.ComponentType,
		ComponentName:    m.ComponentName,
		MethodName:       m.MethodName,
		Type:             v1.DataType(m.Type),
		MethodParameters: m.MethodParameters,
		FileExtension:    m.FileExtension,
		Tags:             m.Tags,
	}
}

// NewCaptureBuffer returns a new Buffer.
func NewCaptureBuffer(
	dir string,
	md *MetaData,
	maxCaptureFileSize int64,
) *CaptureBuffer {
	return &CaptureBuffer{
		Directory:          dir,
		MetaData:           md,
		maxCaptureFileSize: maxCaptureFileSize,
	}
}

// Write writes item onto b. Binary sensor data is written to its own file.
// Tabular data is written to disk in maxCaptureFileSize sized files. Files that
// are still being written to are indicated with the extension
// InProgressFileExt. Files that have finished being written to are indicated by
// FileExt.
func (b *CaptureBuffer) Write(res CaptureResponseWithTimeStamps) error {
	b.lock.Lock()
	defer b.lock.Unlock()

	if res.IsBinary {
		binFile, err := NewCaptureFile(b.Directory, b.MetaData)
		if err != nil {
			return err
		}
		for _, item := range res.ProtoItems() {
			if err := binFile.WriteNext(item); err != nil {
				return err
			}
		}
		if err := binFile.Close(); err != nil {
			return err
		}
		return nil
	}

	if b.nextFile == nil {
		nextFile, err := NewCaptureFile(b.Directory, b.MetaData)
		if err != nil {
			return err
		}
		b.nextFile = nextFile
	} else if b.nextFile.Size() > b.maxCaptureFileSize {
		if err := b.nextFile.Close(); err != nil {
			return err
		}
		nextFile, err := NewCaptureFile(b.Directory, b.MetaData)
		if err != nil {
			return err
		}
		b.nextFile = nextFile
	}

	for _, item := range res.ProtoItems() {
		if err := b.nextFile.WriteNext(item); err != nil {
			return err
		}
	}
	return nil
}

// Flush flushes all buffered data to disk and marks any in progress file as complete.
func (b *CaptureBuffer) Flush() error {
	b.lock.Lock()
	defer b.lock.Unlock()
	if b.nextFile == nil {
		return nil
	}
	if err := b.nextFile.Close(); err != nil {
		return err
	}
	b.nextFile = nil
	return nil
}

// Path returns the path to the directory containing the backing data capture files.
func (b *CaptureBuffer) Path() string {
	return b.Directory
}
