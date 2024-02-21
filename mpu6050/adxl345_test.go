package mpu6050

import (
	"fmt"
	"testing"
	"time"

	"github.com/gurupras/quadcopter"
)

func TestAdxl345(t *testing.T) {
	bus := 1
	dev := quadcopter.NewI2CDevice(ADXL345_ADDRESS, bus)
	adxl345 := NewAdxl345(dev)
	adxl345.Init()
	adxl345.Calibrate(1000, 11*time.Millisecond)
	for {
		vec := adxl345.ReadSample()
		fmt.Printf("X:%v  Y:%v  Z:%v\n", vec.X, vec.Y, vec.Z)

		//x, y, z := adxl345.ReadSampleInG()
		//fmt.Printf("X:%2.2f  Y:%2.2f  Z:%2.2f\n", x, y, z)

		//x, y, z := adxl345.ReadSampleInDegrees()
		//fmt.Printf("X:%2.2f  Y:%2.2f  Z:%2.2f\n", x, y, z)

		time.Sleep(20 * time.Millisecond)
	}
}
