package quadcopter

import (
	"fmt"
	"sync/atomic"
	"time"
)

const (
	MOTOR_BASE    int = 0x29
	MAX_SPEED     int = 240
	DEFAULT_DELAY int = 10
	MAX_DELAY     int = 100
)

type ESC struct {
	*I2CDevice
	CurrentSpeed atomic.Value
	Stop         bool
	initialized  bool
}

func NewESC(dev *I2CDevice) *ESC {
	esc := new(ESC)
	esc.I2CDevice = dev
	return esc
}

func (esc *ESC) Init() {
	if esc.initialized {
		return
	}
	esc.SetAsCurrentDevice()
	for i := 0; i < 1; i++ {
		esc.Write([]byte{0})
	}
	esc.Write([]byte{1})
	esc.CurrentSpeed.Store(0)
	esc.initialized = true
}

func (esc *ESC) AsyncStart() {
	esc.Init()
	prev_speed := 0
	//esc.SetAsCurrentDevice()
	fmt.Sprintf("Starting motor: 0x%X\n", esc.Addr)
	for {
		speed := esc.CurrentSpeed.Load().(int)
		if speed != prev_speed {
			fmt.Println("Attempting to set speed to: %v", speed)
			prev_speed = speed
		}
		esc.Write([]byte{uint8(speed)})
		time.Sleep(25 * time.Millisecond)
	}
}

func (esc *ESC) SlowStop() {
	for {
		speed := esc.CurrentSpeed.Load().(int)
		if speed != 0 {
			speed -= 5
			if speed < 0 {
				speed = 0
			}
			esc.CurrentSpeed.Store(speed)
		} else {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
}

func (esc *ESC) SetSpeed(speed int) {
	if speed > MAX_SPEED {
		speed = MAX_SPEED
	}
	esc.CurrentSpeed.Store(speed)
	//fmt.Sprintf("Speed of motor:0x%X set to %v", esc.Addr, esc.CurrentSpeed)
}

func (esc *ESC) GetSpeed() int {
	return esc.CurrentSpeed.Load().(int)
}
