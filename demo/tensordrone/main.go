/*
How to setup
You must be using a PS3 or compatible controller, along with
a DJI Tello drone to run this example.

You run the Go program on your computer and communicate
wirelessly with the DJI Tello.

How to run

	go run tensordrone/main.go tensorflow_inception_graph.pb imagenet_comp_graph_label_strings.txt

*/

package main

import (
	"bufio"
	"fmt"
	"image"
	"image/color"
	"io"
	"os"
	"os/exec"
	"sync/atomic"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/joystick"
	"gobot.io/x/gobot/platforms/opencv"
	"gocv.io/x/gocv"
)

type pair struct {
	x float64
	y float64
}

var leftX, leftY, rightX, rightY atomic.Value

const (
	offset    = 32767.0
	frameSize = 960 * 720 * 3
)

func main() {
	// parse args
	if len(os.Args) < 3 {
		fmt.Println("How to run:\n\ttensordrone [modelfile] [descriptionsfile]")
		return
	}

	model := os.Args[1]
	descriptions, _ := readDescriptions(os.Args[2])

	joystickAdaptor := joystick.NewAdaptor()
	stick := joystick.NewDriver(joystickAdaptor, "dualshock3")

	drone := tello.NewDriver("8890")

	window := opencv.NewWindowDriver()

	// open Tensorflow DNN classifier
	net := gocv.ReadNetFromTensorflow(model)
	defer net.Close()

	work := func() {
		ffmpeg := exec.Command("ffmpeg", "-i", "pipe:0", "-pix_fmt", "bgr24", "-vcodec", "rawvideo",
			"-an", "-sn", "-s", "960x720", "-f", "rawvideo", "pipe:1")
		ffmpegIn, _ := ffmpeg.StdinPipe()
		ffmpegOut, _ := ffmpeg.StdoutPipe()
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		go func() {
			for {
				buf := make([]byte, frameSize)
				if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
					fmt.Println(err)
					continue
				}

				img := gocv.NewMatFromBytes(720, 960, gocv.MatTypeCV8UC3, buf)
				if img.Empty() {
					continue
				}
				// convert image Mat to 224x224 blob that the classifier can analyze
				blob := gocv.BlobFromImage(img, 1.0, image.Pt(224, 224), gocv.NewScalar(0, 0, 0, 0), true, false)

				// feed the blob into the Tensorflow classifier network
				net.SetInput(blob, "input")

				// run a forward pass thru the network
				prob := net.Forward("softmax2")

				// reshape the results into a 1x1000 matrix
				probMat := prob.Reshape(1, 1)

				// determine the most probable classification, which will be max value
				_, maxVal, _, maxLoc := gocv.MinMaxLoc(probMat)

				// display classification based on position in the descriptions file
				desc := "Unknown"
				if maxLoc.X < 1000 {
					desc = descriptions[maxLoc.X]
				}
				status := fmt.Sprintf("description: %v, maxVal: %v\n", desc, maxVal)
				gocv.PutText(&img, status, image.Pt(10, 20), gocv.FontHersheyPlain, 1.2, color.RGBA{0, 255, 0, 0}, 2)

				blob.Close()
				prob.Close()
				probMat.Close()

				window.ShowImage(img)
				window.WaitKey(1)
			}
		}()

		leftX.Store(float64(0.0))
		leftY.Store(float64(0.0))
		rightX.Store(float64(0.0))
		rightY.Store(float64(0.0))

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
			drone.SetExposure(0)

			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})

		stick.On(joystick.TrianglePress, func(data interface{}) {
			drone.TakeOff()
		})

		stick.On(joystick.XPress, func(data interface{}) {
			drone.Land()
		})

		stick.On(joystick.LeftX, func(data interface{}) {
			val := float64(data.(int16))
			leftX.Store(val)
		})

		stick.On(joystick.LeftY, func(data interface{}) {
			val := float64(data.(int16))
			leftY.Store(val)
		})

		stick.On(joystick.RightX, func(data interface{}) {
			val := float64(data.(int16))
			rightX.Store(val)
		})

		stick.On(joystick.RightY, func(data interface{}) {
			val := float64(data.(int16))
			rightY.Store(val)
		})

		gobot.Every(10*time.Millisecond, func() {
			rightStick := getRightStick()

			switch {
			case rightStick.y < -10:
				drone.Forward(tello.ValidatePitch(rightStick.y, offset))
			case rightStick.y > 10:
				drone.Backward(tello.ValidatePitch(rightStick.y, offset))
			default:
				drone.Forward(0)
			}

			switch {
			case rightStick.x > 10:
				drone.Right(tello.ValidatePitch(rightStick.x, offset))
			case rightStick.x < -10:
				drone.Left(tello.ValidatePitch(rightStick.x, offset))
			default:
				drone.Right(0)
			}
		})

		gobot.Every(10*time.Millisecond, func() {
			leftStick := getLeftStick()
			switch {
			case leftStick.y < -10:
				drone.Up(tello.ValidatePitch(leftStick.y, offset))
			case leftStick.y > 10:
				drone.Down(tello.ValidatePitch(leftStick.y, offset))
			default:
				drone.Up(0)
			}

			switch {
			case leftStick.x > 20:
				drone.Clockwise(tello.ValidatePitch(leftStick.x, offset))
			case leftStick.x < -20:
				drone.CounterClockwise(tello.ValidatePitch(leftStick.x, offset))
			default:
				drone.Clockwise(0)
			}
		})
	}

	robot := gobot.NewRobot("tensordrone",
		[]gobot.Connection{joystickAdaptor},
		[]gobot.Device{stick, drone, window},
		work,
	)

	robot.Start()
}

func getLeftStick() pair {
	s := pair{x: 0, y: 0}
	s.x = leftX.Load().(float64)
	s.y = leftY.Load().(float64)
	return s
}

func getRightStick() pair {
	s := pair{x: 0, y: 0}
	s.x = rightX.Load().(float64)
	s.y = rightY.Load().(float64)
	return s
}

// readDescriptions reads the descriptions from a file
// and returns a slice of its lines.
func readDescriptions(path string) ([]string, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	var lines []string
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		lines = append(lines, scanner.Text())
	}
	return lines, scanner.Err()
}
