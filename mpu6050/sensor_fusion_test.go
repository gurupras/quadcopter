package mpu6050

import (
	"fmt"
	"sync"
	"testing"
	"time"

	"github.com/gurupras/quadcopter"
	"github.com/gurupras/quadcopter/fusion"
)

func TestSensorFusion(t *testing.T) {
	bus := 1
	dev := quadcopter.NewI2CDevice(ADXL345_ADDRESS, bus)
	adxl345 := NewAdxl345(dev)
	adxl345.Init()

	wg := new(sync.WaitGroup)
	wg.Add(2)
	go func() {
		defer wg.Done()
		adxl345.Calibrate(100, 11*time.Millisecond)
	}()

	dev = quadcopter.NewI2CDevice(ITG3200_ADDR, bus)
	itg := NewItg3200(dev)
	itg.Init()

	go func() {
		defer wg.Done()
		itg.Calibrate(100, 11*time.Millisecond)
	}()

	wg.Wait()

	m := New(itg, adxl345)
	sf := fusion.NewMadgwickAHRS(10)
	for {
		acc, gyro := m.ReadSample()
		sf.UpdateIMU(acc.X, acc.Y, acc.Z, gyro.X, gyro.Y, gyro.Z)
		roll, pitch, yaw := sf.GetOrientation()
		fmt.Printf("roll: %.2f  pitch: %.2f  yaw:%.2f\n", roll, pitch, yaw)
		time.Sleep(100 * time.Millisecond)
	}
}
