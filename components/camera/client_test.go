package camera_test

import (
	"bytes"
	"context"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"net"
	"sync"
	"testing"
	"time"

	"github.com/pion/rtp"
	"go.viam.com/test"
	"go.viam.com/utils/rpc"
	"go.viam.com/utils/testutils"
	"google.golang.org/grpc"
	"google.golang.org/grpc/metadata"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/fake"
	"go.viam.com/rdk/components/camera/rtppassthrough"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/gostream/codec/opus"
	"go.viam.com/rdk/gostream/codec/x264"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/robot"
	robotimpl "go.viam.com/rdk/robot/impl"
	"go.viam.com/rdk/robot/web"
	weboptions "go.viam.com/rdk/robot/web/options"
	rdktestutils "go.viam.com/rdk/testutils"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/testutils/robottestutils"
	rutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/utils/contextutils"
)

func TestClient(t *testing.T) {
	logger := logging.NewTestLogger(t)
	listener1, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	injectCamera := &inject.Camera{}
	img := image.NewNRGBA(image.Rect(0, 0, 4, 4))

	var imgBuf bytes.Buffer
	test.That(t, png.Encode(&imgBuf, img), test.ShouldBeNil)
	imgPng, err := png.Decode(bytes.NewReader(imgBuf.Bytes()))
	test.That(t, err, test.ShouldBeNil)

	pcA := pointcloud.New()
	err = pcA.Set(pointcloud.NewVector(5, 5, 5), nil)
	test.That(t, err, test.ShouldBeNil)

	var projA transform.Projector
	intrinsics := &transform.PinholeCameraIntrinsics{ // not the real camera parameters -- fake for test
		Width:  1280,
		Height: 720,
		Fx:     200,
		Fy:     200,
		Ppx:    100,
		Ppy:    100,
	}
	projA = intrinsics

	var imageReleased bool
	var imageReleasedMu sync.Mutex
	// color camera
	injectCamera.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return pcA, nil
	}
	injectCamera.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{
			SupportsPCD:     true,
			IntrinsicParams: intrinsics,
		}, nil
	}
	injectCamera.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return projA, nil
	}
	injectCamera.ImagesFunc = func(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		images := []camera.NamedImage{}
		// one color image
		color := rimage.NewImage(40, 50)
		images = append(images, camera.NamedImage{color, "color"})
		// one depth image
		depth := rimage.NewEmptyDepthMap(10, 20)
		images = append(images, camera.NamedImage{depth, "depth"})
		// a timestamp of 12345
		ts := time.UnixMilli(12345)
		return images, resource.ResponseMetadata{CapturedAt: ts}, nil
	}
	injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
			imageReleasedMu.Lock()
			imageReleased = true
			imageReleasedMu.Unlock()
			return imgPng, func() {}, nil
		})), nil
	}
	// depth camera
	injectCameraDepth := &inject.Camera{}
	depthImg := rimage.NewEmptyDepthMap(10, 20)
	depthImg.Set(0, 0, rimage.Depth(40))
	depthImg.Set(0, 1, rimage.Depth(1))
	depthImg.Set(5, 6, rimage.Depth(190))
	depthImg.Set(9, 12, rimage.Depth(3000))
	depthImg.Set(5, 9, rimage.MaxDepth-rimage.Depth(1))
	injectCameraDepth.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return pcA, nil
	}
	injectCameraDepth.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{
			SupportsPCD:     true,
			IntrinsicParams: intrinsics,
		}, nil
	}
	injectCameraDepth.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return projA, nil
	}
	injectCameraDepth.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
			imageReleasedMu.Lock()
			imageReleased = true
			imageReleasedMu.Unlock()
			return depthImg, func() {}, nil
		})), nil
	}
	// bad camera
	injectCamera2 := &inject.Camera{}
	injectCamera2.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errGeneratePointCloudFailed
	}
	injectCamera2.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, errPropertiesFailed
	}
	injectCamera2.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, errCameraProjectorFailed
	}
	injectCamera2.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errStreamFailed
	}

	resources := map[resource.Name]camera.Camera{
		camera.Named(testCameraName):  injectCamera,
		camera.Named(failCameraName):  injectCamera2,
		camera.Named(depthCameraName): injectCameraDepth,
	}
	cameraSvc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)
	resourceAPI, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, resourceAPI.RegisterRPCService(context.Background(), rpcServer, cameraSvc), test.ShouldBeNil)

	injectCamera.DoFunc = rdktestutils.EchoFunc

	go rpcServer.Serve(listener1)
	defer rpcServer.Stop()

	t.Run("Failing client", func(t *testing.T) {
		cancelCtx, cancel := context.WithCancel(context.Background())
		cancel()
		_, err := viamgrpc.Dial(cancelCtx, listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})

	t.Run("camera client 1", func(t *testing.T) {
		conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldBeNil)
		camera1Client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
		test.That(t, err, test.ShouldBeNil)
		ctx := gostream.WithMIMETypeHint(context.Background(), rutils.MimeTypeRawRGBA)
		frame, _, err := camera.ReadImage(ctx, camera1Client)
		test.That(t, err, test.ShouldBeNil)
		compVal, _, err := rimage.CompareImages(img, frame)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, compVal, test.ShouldEqual, 0) // exact copy, no color conversion
		imageReleasedMu.Lock()
		test.That(t, imageReleased, test.ShouldBeTrue)
		imageReleasedMu.Unlock()

		pcB, err := camera1Client.NextPointCloud(context.Background())
		test.That(t, err, test.ShouldBeNil)
		_, got := pcB.At(5, 5, 5)
		test.That(t, got, test.ShouldBeTrue)

		projB, err := camera1Client.Projector(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, projB, test.ShouldNotBeNil)

		propsB, err := camera1Client.Properties(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, propsB.SupportsPCD, test.ShouldBeTrue)
		test.That(t, propsB.IntrinsicParams, test.ShouldResemble, intrinsics)

		images, meta, err := camera1Client.Images(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, meta.CapturedAt, test.ShouldEqual, time.UnixMilli(12345))
		test.That(t, len(images), test.ShouldEqual, 2)
		test.That(t, images[0].SourceName, test.ShouldEqual, "color")
		test.That(t, images[0].Image.Bounds().Dx(), test.ShouldEqual, 40)
		test.That(t, images[0].Image.Bounds().Dy(), test.ShouldEqual, 50)
		test.That(t, images[0].Image, test.ShouldHaveSameTypeAs, &rimage.LazyEncodedImage{})
		test.That(t, images[0].Image.ColorModel(), test.ShouldHaveSameTypeAs, color.RGBAModel)
		test.That(t, images[1].SourceName, test.ShouldEqual, "depth")
		test.That(t, images[1].Image.Bounds().Dx(), test.ShouldEqual, 10)
		test.That(t, images[1].Image.Bounds().Dy(), test.ShouldEqual, 20)
		test.That(t, images[1].Image, test.ShouldHaveSameTypeAs, &rimage.LazyEncodedImage{})
		test.That(t, images[1].Image.ColorModel(), test.ShouldHaveSameTypeAs, color.Gray16Model)

		// Do
		resp, err := camera1Client.DoCommand(context.Background(), rdktestutils.TestCommand)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, resp["command"], test.ShouldEqual, rdktestutils.TestCommand["command"])
		test.That(t, resp["data"], test.ShouldEqual, rdktestutils.TestCommand["data"])

		test.That(t, camera1Client.Close(context.Background()), test.ShouldBeNil)
		test.That(t, conn.Close(), test.ShouldBeNil)
	})
	t.Run("camera client depth", func(t *testing.T) {
		conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldBeNil)
		client, err := resourceAPI.RPCClient(context.Background(), conn, "", camera.Named(depthCameraName), logger)
		test.That(t, err, test.ShouldBeNil)

		ctx := gostream.WithMIMETypeHint(
			context.Background(), rutils.WithLazyMIMEType(rutils.MimeTypePNG))
		frame, _, err := camera.ReadImage(ctx, client)
		test.That(t, err, test.ShouldBeNil)
		dm, err := rimage.ConvertImageToDepthMap(context.Background(), frame)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, dm, test.ShouldResemble, depthImg)
		imageReleasedMu.Lock()
		test.That(t, imageReleased, test.ShouldBeTrue)
		imageReleasedMu.Unlock()

		test.That(t, client.Close(context.Background()), test.ShouldBeNil)
		test.That(t, conn.Close(), test.ShouldBeNil)
	})

	t.Run("camera client 2", func(t *testing.T) {
		conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldBeNil)
		client2, err := resourceAPI.RPCClient(context.Background(), conn, "", camera.Named(failCameraName), logger)
		test.That(t, err, test.ShouldBeNil)

		_, _, err = camera.ReadImage(context.Background(), client2)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errStreamFailed.Error())

		_, err = client2.NextPointCloud(context.Background())
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errGeneratePointCloudFailed.Error())

		_, err = client2.Projector(context.Background())
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errCameraProjectorFailed.Error())

		_, err = client2.Properties(context.Background())
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errPropertiesFailed.Error())

		test.That(t, conn.Close(), test.ShouldBeNil)
	})
	t.Run("camera client extra", func(t *testing.T) {
		conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldBeNil)

		camClient, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
		test.That(t, err, test.ShouldBeNil)

		injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
			extra, ok := camera.FromContext(ctx)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, extra, test.ShouldBeEmpty)
			return nil, errStreamFailed
		}

		ctx := context.Background()
		_, _, err = camera.ReadImage(ctx, camClient)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errStreamFailed.Error())

		injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
			extra, ok := camera.FromContext(ctx)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, len(extra), test.ShouldEqual, 1)
			test.That(t, extra["hello"], test.ShouldEqual, "world")
			return nil, errStreamFailed
		}

		// one kvp created with camera.Extra
		ext := camera.Extra{"hello": "world"}
		ctx = camera.NewContext(ctx, ext)
		_, _, err = camera.ReadImage(ctx, camClient)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errStreamFailed.Error())

		injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
			extra, ok := camera.FromContext(ctx)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, len(extra), test.ShouldEqual, 1)
			test.That(t, extra[data.FromDMString], test.ShouldBeTrue)

			return nil, errStreamFailed
		}

		// one kvp created with data.FromDMContextKey
		ctx = context.WithValue(context.Background(), data.FromDMContextKey{}, true)
		_, _, err = camera.ReadImage(ctx, camClient)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errStreamFailed.Error())

		injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
			extra, ok := camera.FromContext(ctx)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, len(extra), test.ShouldEqual, 2)
			test.That(t, extra["hello"], test.ShouldEqual, "world")
			test.That(t, extra[data.FromDMString], test.ShouldBeTrue)
			return nil, errStreamFailed
		}

		// merge values from data and camera
		ext = camera.Extra{"hello": "world"}
		ctx = camera.NewContext(ctx, ext)
		_, _, err = camera.ReadImage(ctx, camClient)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, errStreamFailed.Error())

		test.That(t, conn.Close(), test.ShouldBeNil)
	})
}

func TestClientProperties(t *testing.T) {
	logger := logging.NewTestLogger(t)
	listener, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)

	server, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	injectCamera := &inject.Camera{}
	resources := map[resource.Name]camera.Camera{camera.Named(testCameraName): injectCamera}
	svc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)

	rSubType, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, rSubType.RegisterRPCService(context.Background(), server, svc), test.ShouldBeNil)

	go test.That(t, server.Serve(listener), test.ShouldBeNil)
	defer func() { test.That(t, server.Stop(), test.ShouldBeNil) }()

	fakeIntrinsics := &transform.PinholeCameraIntrinsics{
		Width:  1,
		Height: 1,
		Fx:     1,
		Fy:     1,
		Ppx:    1,
		Ppy:    1,
	}
	fakeDistortion := &transform.BrownConrady{
		RadialK1:     1.0,
		RadialK2:     1.0,
		RadialK3:     1.0,
		TangentialP1: 1.0,
		TangentialP2: 1.0,
	}

	testCases := []struct {
		name  string
		props camera.Properties
	}{
		{
			name: "non-nil properties",
			props: camera.Properties{
				SupportsPCD:      true,
				ImageType:        camera.UnspecifiedStream,
				IntrinsicParams:  fakeIntrinsics,
				DistortionParams: fakeDistortion,
			},
		}, {
			name: "nil intrinsic params",
			props: camera.Properties{
				SupportsPCD:      true,
				ImageType:        camera.UnspecifiedStream,
				IntrinsicParams:  nil,
				DistortionParams: fakeDistortion,
			},
		}, {
			name: "nil distortion parameters",
			props: camera.Properties{
				SupportsPCD:      true,
				ImageType:        camera.UnspecifiedStream,
				IntrinsicParams:  fakeIntrinsics,
				DistortionParams: nil,
			},
		}, {
			name:  "empty properties",
			props: camera.Properties{},
		},
	}

	for _, testCase := range testCases {
		t.Run(testCase.name, func(t *testing.T) {
			injectCamera.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return testCase.props, nil
			}

			conn, err := viamgrpc.Dial(context.Background(), listener.Addr().String(), logger)
			test.That(t, err, test.ShouldBeNil)

			client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
			test.That(t, err, test.ShouldBeNil)
			actualProps, err := client.Properties(context.Background())
			test.That(t, err, test.ShouldBeNil)
			test.That(t, actualProps, test.ShouldResemble, testCase.props)

			test.That(t, conn.Close(), test.ShouldBeNil)
		})
	}
}

func TestClientLazyImage(t *testing.T) {
	logger := logging.NewTestLogger(t)
	listener1, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	injectCamera := &inject.Camera{}
	img := image.NewNRGBA64(image.Rect(0, 0, 4, 8))

	var imgBuf bytes.Buffer
	test.That(t, png.Encode(&imgBuf, img), test.ShouldBeNil)
	imgPng, err := png.Decode(bytes.NewReader(imgBuf.Bytes()))
	test.That(t, err, test.ShouldBeNil)

	injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
			mimeType, _ := rutils.CheckLazyMIMEType(gostream.MIMETypeHint(ctx, rutils.MimeTypeRawRGBA))
			switch mimeType {
			case rutils.MimeTypePNG:
				return imgPng, func() {}, nil
			default:
				return nil, nil, errInvalidMimeType
			}
		})), nil
	}

	resources := map[resource.Name]camera.Camera{
		camera.Named(testCameraName): injectCamera,
	}
	cameraSvc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)
	resourceAPI, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, resourceAPI.RegisterRPCService(context.Background(), rpcServer, cameraSvc), test.ShouldBeNil)

	go rpcServer.Serve(listener1)
	defer rpcServer.Stop()

	conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
	test.That(t, err, test.ShouldBeNil)
	camera1Client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
	test.That(t, err, test.ShouldBeNil)

	ctx := gostream.WithMIMETypeHint(context.Background(), rutils.MimeTypePNG)
	frame, _, err := camera.ReadImage(ctx, camera1Client)
	test.That(t, err, test.ShouldBeNil)
	// Should always lazily decode
	test.That(t, frame, test.ShouldHaveSameTypeAs, &rimage.LazyEncodedImage{})
	frameLazy := frame.(*rimage.LazyEncodedImage)
	test.That(t, frameLazy.RawData(), test.ShouldResemble, imgBuf.Bytes())

	ctx = gostream.WithMIMETypeHint(context.Background(), rutils.WithLazyMIMEType(rutils.MimeTypePNG))
	frame, _, err = camera.ReadImage(ctx, camera1Client)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, frame, test.ShouldHaveSameTypeAs, &rimage.LazyEncodedImage{})
	frameLazy = frame.(*rimage.LazyEncodedImage)
	test.That(t, frameLazy.RawData(), test.ShouldResemble, imgBuf.Bytes())

	test.That(t, frameLazy.MIMEType(), test.ShouldEqual, rutils.MimeTypePNG)
	compVal, _, err := rimage.CompareImages(img, frame)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, compVal, test.ShouldEqual, 0) // exact copy, no color conversion

	test.That(t, conn.Close(), test.ShouldBeNil)
}

func TestClientWithInterceptor(t *testing.T) {
	// Set up gRPC server
	logger := logging.NewTestLogger(t)
	listener1, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	// Set up camera that adds timestamps into the gRPC response header.
	injectCamera := &inject.Camera{}

	pcA := pointcloud.New()
	err = pcA.Set(pointcloud.NewVector(5, 5, 5), nil)
	test.That(t, err, test.ShouldBeNil)

	k, v := "hello", "world"
	injectCamera.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		var grpcMetadata metadata.MD = make(map[string][]string)
		grpcMetadata.Set(k, v)
		grpc.SendHeader(ctx, grpcMetadata)
		return pcA, nil
	}

	// Register CameraService API in our gRPC server.
	resources := map[resource.Name]camera.Camera{
		camera.Named(testCameraName): injectCamera,
	}
	cameraSvc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)
	resourceAPI, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, resourceAPI.RegisterRPCService(context.Background(), rpcServer, cameraSvc), test.ShouldBeNil)

	// Start serving requests.
	go rpcServer.Serve(listener1)
	defer rpcServer.Stop()

	// Set up gRPC client with context with metadata interceptor.
	conn, err := viamgrpc.Dial(
		context.Background(),
		listener1.Addr().String(),
		logger,
		rpc.WithUnaryClientInterceptor(contextutils.ContextWithMetadataUnaryClientInterceptor),
	)
	test.That(t, err, test.ShouldBeNil)
	camera1Client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
	test.That(t, err, test.ShouldBeNil)

	// Construct a ContextWithMetadata to pass into NextPointCloud and check that the
	// interceptor correctly injected the metadata from the gRPC response header into the
	// context.
	ctx, md := contextutils.ContextWithMetadata(context.Background())
	pcB, err := camera1Client.NextPointCloud(ctx)
	test.That(t, err, test.ShouldBeNil)
	_, got := pcB.At(5, 5, 5)
	test.That(t, got, test.ShouldBeTrue)
	test.That(t, md[k][0], test.ShouldEqual, v)

	test.That(t, conn.Close(), test.ShouldBeNil)
}

func TestClientStreamAfterClose(t *testing.T) {
	// Set up gRPC server
	logger := logging.NewTestLogger(t)
	listener, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	// Set up camera that can stream images
	img := image.NewNRGBA(image.Rect(0, 0, 4, 4))
	injectCamera := &inject.Camera{}
	injectCamera.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	injectCamera.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return gostream.NewEmbeddedVideoStreamFromReader(gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
			return img, func() {}, nil
		})), nil
	}

	// Register CameraService API in our gRPC server.
	resources := map[resource.Name]camera.Camera{
		camera.Named(testCameraName): injectCamera,
	}
	cameraSvc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)
	resourceAPI, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, resourceAPI.RegisterRPCService(context.Background(), rpcServer, cameraSvc), test.ShouldBeNil)

	// Start serving requests.
	go rpcServer.Serve(listener)
	defer rpcServer.Stop()

	// Make client connection
	conn, err := viamgrpc.Dial(context.Background(), listener.Addr().String(), logger)
	test.That(t, err, test.ShouldBeNil)
	client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(testCameraName), logger)
	test.That(t, err, test.ShouldBeNil)

	// Get a stream
	stream, err := client.Stream(context.Background())
	test.That(t, stream, test.ShouldNotBeNil)
	test.That(t, err, test.ShouldBeNil)

	// Read from stream
	media, _, err := stream.Next(context.Background())
	test.That(t, media, test.ShouldNotBeNil)
	test.That(t, err, test.ShouldBeNil)

	// Close client and read from stream
	test.That(t, client.Close(context.Background()), test.ShouldBeNil)
	media, _, err = stream.Next(context.Background())
	test.That(t, media, test.ShouldBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "context canceled")

	// Get a new stream
	stream, err = client.Stream(context.Background())
	test.That(t, stream, test.ShouldNotBeNil)
	test.That(t, err, test.ShouldBeNil)

	// Read from the new stream
	media, _, err = stream.Next(context.Background())
	test.That(t, media, test.ShouldNotBeNil)
	test.That(t, err, test.ShouldBeNil)

	// Close client and connection
	test.That(t, client.Close(context.Background()), test.ShouldBeNil)
	test.That(t, conn.Close(), test.ShouldBeNil)
}

// See modmanager_test.go for the happy path (aka, when the
// client has a webrtc connection).
func TestRTPPassthroughWithoutWebRTC(t *testing.T) {
	logger := logging.NewTestLogger(t)
	camName := "rtp_passthrough_camera"
	listener1, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger.AsZap(), rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	injectCamera := &inject.Camera{}
	resources := map[resource.Name]camera.Camera{
		camera.Named(camName): injectCamera,
	}
	cameraSvc, err := resource.NewAPIResourceCollection(camera.API, resources)
	test.That(t, err, test.ShouldBeNil)
	resourceAPI, ok, err := resource.LookupAPIRegistration[camera.Camera](camera.API)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, resourceAPI.RegisterRPCService(context.Background(), rpcServer, cameraSvc), test.ShouldBeNil)

	go rpcServer.Serve(listener1)
	defer rpcServer.Stop()

	t.Run("Failing client", func(t *testing.T) {
		cancelCtx, cancel := context.WithCancel(context.Background())
		cancel()
		_, err := viamgrpc.Dial(cancelCtx, listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})

	t.Run("rtp_passthrough client fails without webrtc connection", func(t *testing.T) {
		conn, err := viamgrpc.Dial(context.Background(), listener1.Addr().String(), logger)
		test.That(t, err, test.ShouldBeNil)
		camera1Client, err := camera.NewClientFromConn(context.Background(), conn, "", camera.Named(camName), logger)
		test.That(t, err, test.ShouldBeNil)
		rtpPassthroughClient, ok := camera1Client.(rtppassthrough.Source)
		test.That(t, ok, test.ShouldBeTrue)
		sub, err := rtpPassthroughClient.SubscribeRTP(context.Background(), 512, func(pkts []*rtp.Packet) {
			t.Log("should not be called")
			t.FailNow()
		})
		test.That(t, err, test.ShouldBeError, camera.ErrNoPeerConnection)
		test.That(t, sub, test.ShouldResemble, rtppassthrough.NilSubscription)
		err = rtpPassthroughClient.Unsubscribe(context.Background(), rtppassthrough.NilSubscription.ID)
		test.That(t, err, test.ShouldBeError, camera.ErrNoPeerConnection)

		test.That(t, conn.Close(), test.ShouldBeNil)
	})
}

var (
	Green   = "\033[32m"
	Red     = "\033[31m"
	Magenta = "\033[35m"
	Cyan    = "\033[36m"
	Yellow  = "\033[33m"
	Reset   = "\033[0m"
)

// this helps make the test case much easier to read.
func greenLog(t *testing.T, msg string) {
	t.Log(Green + msg + Reset)
}

func TestReconnectToRemoveAfterReboot(t *testing.T) {
	logger := logging.NewTestLogger(t).Sublogger(t.Name())
	ctx := context.Background()

	remoteCfg := &config.Config{
		Components: []resource.Config{
			{
				Name:  "rtpPassthroughCamera",
				API:   resource.NewAPI("rdk", "component", "camera"),
				Model: resource.DefaultModelFamily.WithModel("fake"),
				ConvertedAttributes: &fake.Config{
					RTPPassthrough: true,
				},
			},
		},
	}

	remote, err := robotimpl.New(ctx, remoteCfg, logger.Sublogger("remote"), robotimpl.WithViamHomeDir(t.TempDir()))
	test.That(t, err, test.ShouldBeNil)

	options, listner, addr := robottestutils.CreateBaseOptionsAndListener(t)
	test.That(t, remote.StartWeb(ctx, options), test.ShouldBeNil)

	mainCfg := &config.Config{
		Remotes: []config.Remote{
			{
				Name:     "remote",
				Address:  addr,
				Insecure: true,
			},
		},
	}

	main, err := robotimpl.New(ctx, mainCfg, logger.Sublogger("main"), robotimpl.WithViamHomeDir(t.TempDir()))
	test.That(t, err, test.ShouldBeNil)
	defer main.Close(ctx)

	greenLog(t, "robot setup")

	greenLog(t, fmt.Sprintf("ResourceRPCAPIs before close: %#v", main.ResourceRPCAPIs()))
	greenLog(t, fmt.Sprintf("ResourceNames before close: %#v", main.ResourceNames()))

	expectedResources := []resource.Name{
		camera.Named("remote:rtpPassthroughCamera"),
	}
	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	cameraClient, err := camera.FromRobot(main, "remote:rtpPassthroughCamera")
	test.That(t, err, test.ShouldBeNil)

	image, _, err := cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, image, test.ShouldNotBeNil)
	greenLog(t, "got images")

	greenLog(t, "calling close on remote")
	test.That(t, remote.Close(ctx), test.ShouldBeNil)
	greenLog(t, "close called on remote")

	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), []resource.Name{})
	})

	greenLog(t, "confirming images returns an error")
	_, _, err = cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeError)

	remote2, err := robotimpl.New(ctx, remoteCfg, logger.Sublogger("remote-post-reboot"), robotimpl.WithViamHomeDir(t.TempDir()))
	test.That(t, err, test.ShouldBeNil)
	defer func() { test.That(t, remote2.Close(ctx), test.ShouldBeNil) }()

	// Note: There's a slight chance this test can fail if someone else
	// claims the port we just released by closing the server.
	listner, err = net.Listen("tcp", listner.Addr().String())
	test.That(t, err, test.ShouldBeNil)
	options.Network.Listener = listner
	err = remote2.StartWeb(ctx, options)
	test.That(t, err, test.ShouldBeNil)

	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	greenLog(t, "confirming images is returns success")
	timeoutCtx, timeoutFn := context.WithTimeout(context.Background(), time.Second*10)
	defer timeoutFn()
	for timeoutCtx.Err() == nil {
		_, _, err = cameraClient.Images(ctx)
		if err == nil {
			break
		}
		t.Log(err.Error())
		time.Sleep(time.Millisecond * 50)
	}
	test.That(t, timeoutCtx.Err(), test.ShouldBeNil)
}

func TestReconnectToRemoveAfterReboot2(t *testing.T) {
	logger := logging.NewTestLogger(t).Sublogger(t.Name())
	ctx := context.Background()

	remoteCfg1 := &config.Config{
		Components: []resource.Config{
			{
				Name:  "rtpPassthroughCamera",
				API:   resource.NewAPI("rdk", "component", "camera"),
				Model: resource.DefaultModelFamily.WithModel("fake"),
				ConvertedAttributes: &fake.Config{
					RTPPassthrough: true,
				},
			},
		},
	}

	optionsRemote, listnerRemote, addrRemote := robottestutils.CreateBaseOptionsAndListener(t)
	remoteRobot1, remoteWebSvc1 := setupStreamingRobotWithOptions(t, remoteCfg1, logger.Sublogger("remote-1"), optionsRemote)

	mainCfg := &config.Config{
		Remotes: []config.Remote{
			{
				Name:     "remote",
				Address:  addrRemote,
				Insecure: true,
			},
		},
	}

	optionsMain, _, _ := robottestutils.CreateBaseOptionsAndListener(t)
	main, mainWebSvc := setupStreamingRobotWithOptions(t, mainCfg, logger.Sublogger("main"), optionsMain)
	greenLog(t, "robot setup")
	defer main.Close(ctx)
	defer mainWebSvc.Close(ctx)

	expectedResources := []resource.Name{
		camera.Named("remote:rtpPassthroughCamera"),
	}
	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 30, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	greenLog(t, fmt.Sprintf("ResourceRPCAPIs before close: %#v", main.ResourceRPCAPIs()))
	greenLog(t, fmt.Sprintf("ResourceNames before close: %#v", main.ResourceNames()))

	cameraClient, err := camera.FromRobot(main, "remote:rtpPassthroughCamera")
	test.That(t, err, test.ShouldBeNil)

	image, _, err := cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, image, test.ShouldNotBeNil)
	greenLog(t, "got images")

	greenLog(t, "calling close")
	test.That(t, remoteRobot1.Close(ctx), test.ShouldBeNil)
	test.That(t, remoteWebSvc1.Close(ctx), test.ShouldBeNil)
	greenLog(t, "close called")

	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 30, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), []resource.Name{})
	})

	greenLog(t, "confirming images returns an error")
	_, _, err = cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeError)

	// bind second instance of remote to the same port as the first
	listnerRemote, err = net.Listen("tcp", listnerRemote.Addr().String())
	test.That(t, err, test.ShouldBeNil)
	optionsRemote.Network.Listener = listnerRemote
	remote2Second, remote2SecondWeb := setupStreamingRobotWithOptions(t, remoteCfg1, logger.Sublogger("remote-second"), optionsRemote)
	defer remote2Second.Close(ctx)
	defer remote2SecondWeb.Close(ctx)

	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	greenLog(t, "confirming images is returns success")
	timeoutCtx, timeoutFn := context.WithTimeout(context.Background(), time.Second*10)
	defer timeoutFn()
	for timeoutCtx.Err() == nil {
		_, _, err = cameraClient.Images(ctx)
		if err == nil {
			break
		}
		t.Log(err.Error())
		time.Sleep(time.Millisecond * 50)
	}
	test.That(t, timeoutCtx.Err(), test.ShouldBeNil)
}

func setupStreamingRobotWithOptions(
	t *testing.T,
	robotConfig *config.Config,
	logger logging.Logger,
	options weboptions.Options,
) (robot.LocalRobot, web.Service) {
	t.Helper()

	ctx := context.Background()
	robot, err := robotimpl.RobotFromConfig(ctx, robotConfig, logger)
	test.That(t, err, test.ShouldBeNil)

	// We initialize with a stream config such that the stream server is capable of creating video stream and
	// audio stream data.
	webSvc := web.New(robot, logger, web.WithStreamConfig(gostream.StreamConfig{
		AudioEncoderFactory: opus.NewEncoderFactory(),
		VideoEncoderFactory: x264.NewEncoderFactory(),
	}))
	err = webSvc.Start(ctx, options)
	test.That(t, err, test.ShouldBeNil)

	return robot, webSvc
}

// TODO: Get this working
func TestRemoteUnreachableTriggersClientClose(t *testing.T) {
	logger := logging.NewTestLogger(t).Sublogger(t.Name())
	ctx := context.Background()

	remoteCfg2 := &config.Config{
		Components: []resource.Config{
			{
				Name:  "rtpPassthroughCamera",
				API:   resource.NewAPI("rdk", "component", "camera"),
				Model: resource.DefaultModelFamily.WithModel("fake"),
				ConvertedAttributes: &fake.Config{
					RTPPassthrough: true,
				},
			},
		},
	}

	optionsRemote2, listnerRemote2, addrRemote2 := robottestutils.CreateBaseOptionsAndListener(t)
	remote2, remoteWebSvc2 := setupStreamingRobotWithOptions(t, remoteCfg2, logger.Sublogger("remote-2"), optionsRemote2)

	remoteCfg1 := &config.Config{
		Remotes: []config.Remote{
			{
				Name:     "remote-2",
				Address:  addrRemote2,
				Insecure: true,
			},
		},
	}

	optionsRemote1, _, addrRemote1 := robottestutils.CreateBaseOptionsAndListener(t)
	remote1, remoteWebSvc1 := setupStreamingRobotWithOptions(t, remoteCfg1, logger.Sublogger("remote-1"), optionsRemote1)
	defer remote1.Close(ctx)
	defer remoteWebSvc1.Close(ctx)

	mainCfg := &config.Config{
		Remotes: []config.Remote{
			{
				Name:     "remote-1",
				Address:  addrRemote1,
				Insecure: true,
			},
		},
	}

	optionsMain, _, _ := robottestutils.CreateBaseOptionsAndListener(t)
	main, mainWebSvc := setupStreamingRobotWithOptions(t, mainCfg, logger.Sublogger("remote-1"), optionsMain)
	defer main.Close(ctx)
	defer mainWebSvc.Close(ctx)
	greenLog(t, "robot setup")

	expectedResources := []resource.Name{
		camera.Named("remote-1:remote-2:rtpPassthroughCamera"),
	}
	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 30, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	greenLog(t, fmt.Sprintf("ResourceRPCAPIs before close: %#v", main.ResourceRPCAPIs()))
	greenLog(t, fmt.Sprintf("ResourceNames before close: %#v", main.ResourceNames()))

	cameraClient, err := camera.FromRobot(main, "remote-1:remote-2:rtpPassthroughCamera")
	test.That(t, err, test.ShouldBeNil)

	image, _, err := cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, image, test.ShouldNotBeNil)
	greenLog(t, "got images")

	greenLog(t, "calling close")
	test.That(t, remote2.Close(ctx), test.ShouldBeNil)
	test.That(t, remoteWebSvc2.Close(ctx), test.ShouldBeNil)
	greenLog(t, "close called")

	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), []resource.Name{})
	})

	_, _, err = cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeError)

	listnerRemote2, err = net.Listen("tcp", listnerRemote2.Addr().String())
	test.That(t, err, test.ShouldBeNil)
	optionsRemote2.Network.Listener = listnerRemote2
	remote2Second, remote2SecondWeb := setupStreamingRobotWithOptions(t, remoteCfg2, logger.Sublogger("remote-second"), optionsRemote2)
	defer remote2Second.Close(ctx)
	defer remote2SecondWeb.Close(ctx)

	greenLog(t, "waiting for second instance to come up")
	testutils.WaitForAssertionWithSleep(t, time.Millisecond*100, 300, func(tb testing.TB) {
		rdktestutils.VerifySameResourceNames(tb, main.ResourceNames(), expectedResources)
	})

	image, _, err = cameraClient.Images(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, image, test.ShouldNotBeNil)
}
