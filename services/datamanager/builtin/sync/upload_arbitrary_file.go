package sync

import (
	"context"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"github.com/benbjohnson/clock"
	"github.com/pkg/errors"
	v1 "go.viam.com/api/app/datasync/v1"

	"go.viam.com/rdk/logging"
)

// UploadChunkSize defines the size of the data included in each message of a FileUpload stream.
var UploadChunkSize = 64 * 1024

func uploadArbitraryFile(
	ctx context.Context,
	f *os.File,
	conn cloudConn,
	tags []string,
	fileLastModifiedMillis int,
	clock clock.Clock,
	logger logging.Logger,
) error {
	logger.Debugf("attempting to sync arbitrary file: %s", f.Name())
	path, err := filepath.Abs(f.Name())
	if err != nil {
		return errors.Wrapf(err, "failed to get absolute path for arbitrary file %s", f.Name())
	}

	// Only sync non-datacapture files that have not been modified in the last
	// fileLastModifiedMillis to avoid uploading files that are being
	// to written to.
	info, err := os.Stat(path)
	if err != nil {
		return errors.Wrapf(err, "stat failed for arbitrary file %s", path)
	}
	if info.Size() == 0 {
		return fmt.Errorf("arbitrary file is empty (0 bytes): %s", path)
	}

	timeSinceMod := clock.Since(info.ModTime())
	if timeSinceMod < time.Duration(fileLastModifiedMillis)*time.Millisecond {
		return fmt.Errorf("arbitrary file modified too recently: %s", path)
	}

	logger.Debugf("datasync.FileUpload request started for arbitrary file: %s", path)
	stream, err := conn.client.FileUpload(ctx)
	if err != nil {
		return errors.Wrapf(err, "error creating FileUpload client for arbitrary file: %s", path)
	}

	// Send metadata FileUploadRequest.
	logger.Debugf("datasync.FileUpload request sending metadata for arbitrary file: %s", path)
	if err := stream.Send(&v1.FileUploadRequest{
		UploadPacket: &v1.FileUploadRequest_Metadata{
			Metadata: &v1.UploadMetadata{
				PartId:        conn.partID,
				Type:          v1.DataType_DATA_TYPE_FILE,
				FileName:      path,
				FileExtension: filepath.Ext(f.Name()),
				Tags:          tags,
			},
		},
	}); err != nil {
		return errors.Wrapf(err, "FileUpload failed sending metadata for arbitrary file %s", path)
	}

	if err := sendFileUploadRequests(ctx, stream, f, path, logger); err != nil {
		return errors.Wrapf(err, "FileUpload failed to sync arbitrary file: %s", path)
	}

	logger.Debugf("datasync.FileUpload closing for arbitrary file: %s", path)
	_, err = stream.CloseAndRecv()
	return errors.Wrapf(err, "FileUpload failed to CloseAndRecv syncing arbitrary file %s", path)
}

func sendFileUploadRequests(
	ctx context.Context,
	stream v1.DataSyncService_FileUploadClient,
	f *os.File,
	path string,
	logger logging.Logger,
) error {
	// Loop until there is no more content to be read from file.
	i := 0
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		// Get the next UploadRequest from the file.
		uploadReq, err := getNextFileUploadRequest(f)

		// EOF means we've completed successfully.
		if errors.Is(err, io.EOF) {
			return nil
		}

		if err != nil {
			return err
		}

		logger.Debugf("datasync.FileUpload sending chunk %d for file: %s", i, path)
		if err = stream.Send(uploadReq); err != nil {
			return err
		}
		i++
	}
}

func getNextFileUploadRequest(f *os.File) (*v1.FileUploadRequest, error) {
	// Get the next file data reading from file, check for an error.
	next, err := readNextFileChunk(f)
	if err != nil {
		return nil, err
	}
	// Otherwise, return an UploadRequest and no error.
	return &v1.FileUploadRequest{
		UploadPacket: &v1.FileUploadRequest_FileContents{
			FileContents: next,
		},
	}, nil
}

func readNextFileChunk(f *os.File) (*v1.FileData, error) {
	byteArr := make([]byte, UploadChunkSize)
	numBytesRead, err := f.Read(byteArr)
	if err != nil {
		return nil, err
	}
	return &v1.FileData{Data: byteArr[:numBytesRead]}, nil
}
