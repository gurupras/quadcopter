package mpu9250

import (
	"fmt"
	"testing"
	"time"

	"github.com/gurupras/quadcopter"
	"github.com/gurupras/quadcopter/fusion"
	log "github.com/sirupsen/logrus"
	"github.com/stretchr/testify/require"
)

func TestMPU9250Basic(t *testing.T) {
	log.SetLevel(log.DebugLevel)

	require := require.New(t)

	dev := New(1)

	err := dev.Init()
	require.Nil(err)

	last := time.Now()

	acc := &quadcopter.Vec3{}
	gyro := &quadcopter.Vec3{}
	mag := &quadcopter.Vec3{}

	sampleInterval := 10 * time.Millisecond
	updateInterval := 50 * time.Millisecond
	updateRate := float64(1 * time.Second / updateInterval)
	numSamplesPerUpdate := float64(updateInterval / sampleInterval)

	sampleTicker := time.Tick(sampleInterval)
	updateTicker := time.Tick(updateInterval)

	log.Debugf("SampleRate=%vHZ\n", updateRate)
	sf := fusion.NewMadgwickAHRS(updateRate)

	for {
		select {
		case <-sampleTicker:
			a, g, m := dev.ReadSample()
			acc.Add(&a)
			gyro.Add(&g)
			mag.Add(&m)
			sf.Update(a.X, a.Y, a.Z, g.X, g.Y, g.Z, m.X, m.Y, m.Z)
		case <-updateTicker:
			acc.Divide(numSamplesPerUpdate)
			gyro.Divide(numSamplesPerUpdate)
			mag.Divide(numSamplesPerUpdate)

			// sf.Update(acc.X, acc.Y, acc.Z, gyro.X, gyro.Y, gyro.Z, mag.X, mag.Y, mag.Z)
			// sf.UpdateIMU(acc.X, acc.Y, acc.Z, gyro.X, gyro.Y, gyro.Z)

			now := time.Now()

			roll, pitch, yaw := sf.GetOrientation()
			if now.Sub(last).Seconds() > 1 {
				last = now
				go func(acc, gyro, mag quadcopter.Vec3, roll quadcopter.Roll, pitch quadcopter.Pitch, yaw quadcopter.Yaw) {
					data := make([]string, 0)
					data = append(data, fmt.Sprintf("Accelerometer: x=%v y=%v z=%v\n", round(acc.X), round(acc.Y), round(acc.Z)))
					data = append(data, fmt.Sprintf("Gyroscope:     x=%v y=%v z=%v\n", round(gyro.X), round(gyro.Y), round(gyro.Z)))
					data = append(data, fmt.Sprintf("Magnetometer:  x=%v y=%v z=%v\n", round(mag.X), round(mag.Y), round(mag.Z)))

					log.Debugln("------------------------------")
					for _, str := range data {
						log.Debugln(str)
					}
					log.Debugf("roll:  %.2f\n", roll)
					log.Debugf("pitch: %.2f\n", pitch)
					log.Debugf("yaw:   %.2f\n", yaw)
					log.Debugln("------------------------------")
				}(*acc, *gyro, *mag, roll, pitch, yaw)
			}
			acc.Reset()
			gyro.Reset()
			mag.Reset()
		}
	}
}

func TestMPU9250Calibration(t *testing.T) {
	log.SetLevel(log.DebugLevel)
	require := require.New(t)

	dev := New(1)

	err := dev.Calibrate()
	require.Nil(err)
}
