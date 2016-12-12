package quadcopter

import "testing"
import "time"
import "fmt"

func TestItg3200(t *testing.T) {
	bus := 1
	dev := NewI2CDevice(ITG3200_ADDR, bus)
	itg := NewItg3200(dev)
	itg.Init()
	itg.Calibrate(1000, 11*time.Millisecond)
	for {
		//x, y, z := itg.ReadSampleInDegrees()
		//fmt.Printf("X:%2.2f  Y:%2.2f  Z:%2.2f\n", x, y, z)
		x, y, z := itg.ReadSample()
		fmt.Printf("X:%v  Y:%v  Z:%v\n", x, y, z)
		time.Sleep(100 * time.Millisecond)
	}
}
