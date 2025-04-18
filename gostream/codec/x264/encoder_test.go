package x264

import (
	"bytes"
	"context"
	"image"
	"image/color"
	"image/jpeg"
	"image/png"
	"os"
	"testing"

	"github.com/nfnt/resize"
	"go.viam.com/test"

	"go.viam.com/rdk/logging"
)

const (
	DefaultKeyFrameInterval = 30
	Width                   = 640
	Height                  = 480
)

func pngToImage(b *testing.B, loc string) (image.Image, error) {
	b.Helper()
	openBytes, err := os.ReadFile(loc)
	test.That(b, err, test.ShouldBeNil)
	return png.Decode(bytes.NewReader(openBytes))
}

func resizeImg(b *testing.B, img image.Image, width, height uint) image.Image {
	b.Helper()
	newImage := resize.Resize(width, height, img, resize.Lanczos3)
	return newImage
}

func convertToYCbCr(b *testing.B, src image.Image) (image.Image, error) {
	b.Helper()
	bf := new(bytes.Buffer)
	err := jpeg.Encode(bf, src, nil)
	test.That(b, err, test.ShouldBeNil)
	dst, _, err := image.Decode(bf)
	test.That(b, err, test.ShouldBeNil)
	test.That(b, dst.ColorModel(), test.ShouldResemble, color.YCbCrModel)
	return dst, err
}

func getResizedImageFromFile(b *testing.B, loc string) image.Image {
	b.Helper()
	img, err := pngToImage(b, loc)
	test.That(b, err, test.ShouldBeNil)
	return resizeImg(b, img, uint(Width), uint(Height))
}

func BenchmarkEncodeRGBA(b *testing.B) {
	var w bool
	var logger logging.Logger

	imgCyan := getResizedImageFromFile(b, "../../data/cyan.png")
	imgFuchsia := getResizedImageFromFile(b, "../../data/fuchsia.png")
	ctx := context.Background()
	encoder, err := NewEncoder(Width, Height, DefaultKeyFrameInterval, logger)
	test.That(b, err, test.ShouldBeNil)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if w {
			_, err = encoder.Encode(ctx, imgCyan)
			test.That(b, err, test.ShouldBeNil)
		} else {
			_, err = encoder.Encode(ctx, imgFuchsia)
			test.That(b, err, test.ShouldBeNil)
		}
		w = !w
	}
}

func BenchmarkEncodeYCbCr(b *testing.B) {
	var w bool
	var logger logging.Logger
	imgCyan := getResizedImageFromFile(b, "../../data/cyan.png")
	imgFuchsia := getResizedImageFromFile(b, "../../data/fuchsia.png")

	imgFY, err := convertToYCbCr(b, imgFuchsia)
	test.That(b, err, test.ShouldBeNil)

	imgCY, err := convertToYCbCr(b, imgCyan)
	test.That(b, err, test.ShouldBeNil)

	encoder, err := NewEncoder(Width, Height, DefaultKeyFrameInterval, logger)
	test.That(b, err, test.ShouldBeNil)

	ctx := context.Background()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if w {
			_, err = encoder.Encode(ctx, imgFY)
			test.That(b, err, test.ShouldBeNil)
		} else {
			_, err = encoder.Encode(ctx, imgCY)
			test.That(b, err, test.ShouldBeNil)
		}
		w = !w
	}
}

func TestCalcBitrateFromResolution(t *testing.T) {
	bitrateTests := []struct {
		width, height int
		framerate     float32
		expected      int
	}{
		{640, 480, 30, 1382400},
		{1920, 1080, 30, 9331200},
		{3840, 2160, 30, maxBitrate},
		{240, 180, 30, minBitrate},
	}

	for _, bt := range bitrateTests {
		t.Run("", func(t *testing.T) {
			bitrate := calcBitrateFromResolution(bt.width, bt.height, bt.framerate)
			test.That(t, bitrate, test.ShouldEqual, bt.expected)
		})
	}
}
