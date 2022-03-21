package mpu6050

import (
	"fmt"
	"sync"
	"testing"
	"time"

	"github.com/gurupras/quadcopter"
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

	sf := NewSensorFusion(itg, adxl345)
	for {
		x, y, z := sf.ReadSampleInDegrees()
		fmt.Printf("X:%d  Y:%d  Z:%d\n", int(x), int(y), int(z))

		time.Sleep(100 * time.Millisecond)
	}
}
