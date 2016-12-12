package quadcopter

import "testing"
import "time"
import "fmt"

func TestSensorFusion(t *testing.T) {
	bus := 1
	dev := NewI2CDevice(ADXL345_ADDRESS, bus)
	adxl345 := NewAdxl345(dev)
	adxl345.Init()
	adxl345.Calibrate(1000, 11*time.Millisecond)

	dev = NewI2CDevice(ITG3200_ADDR, bus)
	itg := NewItg3200(dev)
	itg.Init()
	itg.Calibrate(1000, 11*time.Millisecond)

	sf := NewSensorFusion(itg, adxl345)
	for {
		x, y, z := adxl345.ReadSample()
		fmt.Printf("X:%v  Y:%v  Z:%v\n", x, y, z)

		//x, y, z := adxl345.ReadSampleInG()
		//fmt.Printf("X:%2.2f  Y:%2.2f  Z:%2.2f\n", x, y, z)

		//x, y, z := adxl345.ReadSampleInDegrees()
		//fmt.Printf("X:%2.2f  Y:%2.2f  Z:%2.2f\n", x, y, z)

		time.Sleep(20 * time.Millisecond)
	}
}
