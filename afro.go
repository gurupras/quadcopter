package quadcopter

import (
	"sync"
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
	CurrentSpeed int
	SpeedLock    sync.Mutex
	Stop         bool
}

func NewESC(dev *I2CDevice) *ESC {
	esc := new(ESC)
	esc.I2CDevice = dev
	return esc
}

func (esc *ESC) Init() {
	esc.SetAsCurrentDevice()
	esc.Write([]byte{0})
	esc.CurrentSpeed = 1
}

func (esc *ESC) AsyncStart() {
	esc.Init()
	prev_speed := 0
	esc.SetAsCurrentDevice()

	for {
		esc.SpeedLock.Lock()
		speed := esc.CurrentSpeed
		if speed != prev_speed {
			esc.Write([]byte{uint8(speed)})
		}
		time.Sleep(10 * time.Millisecond)
		esc.SpeedLock.Unlock()
	}
}

func (esc *ESC) SlowStop() {
	for {
		esc.SpeedLock.Lock()
		if esc.CurrentSpeed != 0 {
			esc.CurrentSpeed -= 5
			if esc.CurrentSpeed < 0 {
				esc.CurrentSpeed = 0
			}
			esc.SpeedLock.Unlock()
		} else {
			esc.SpeedLock.Unlock()
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
}

func (esc *ESC) SetSpeed(speed int) {
	if speed > MAX_SPEED {
		speed = MAX_SPEED
	}
	esc.SpeedLock.Lock()
	esc.CurrentSpeed = speed
	esc.SpeedLock.Unlock()
}
