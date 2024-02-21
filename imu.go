package quadcopter

import (
	"github.com/gurupras/go-i2c"
)

type Axis int

const (
	AXIS_X Axis = iota
	AXIS_Y Axis = iota
	AXIS_Z Axis = iota
)

type Roll float64
type Pitch float64
type Yaw float64

type AHRS interface {
	GetOrientation() (Roll, Pitch, Yaw)
}

type Sensor interface {
	ReadSample() Vec3
	ReadSampleInDegrees() (float64, float64, float64)
}

type IMU struct {
	XOffset int16
	YOffset int16
	ZOffset int16
}

func ReadAxis(dev *i2c.I2C, highReg, lowReg uint8, offset int16) int16 {
	var (
		h   uint8 = 0
		l   uint8 = 0
		err error
	)

	if h, err = dev.ReadRegU8(highReg); err != nil {
		return 0
	}
	if l, err = dev.ReadRegU8(lowReg); err != nil {
		return 0
	}

	h16 := uint16(h) << 8
	l16 := uint16(l)
	v16 := h16 | l16
	val := int32(v16) - int32(offset)
	if val > 32768 {
		val -= 65536
	}
	return int16(val)
}
