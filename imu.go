package quadcopter

import "github.com/gurupras/go-i2c"

type Axis int

const (
	AXIS_X Axis = iota
	AXIS_Y Axis = iota
	AXIS_Z Axis = iota
)

type IMU struct {
	XOffset int16
	YOffset int16
	ZOffset int16

	REG_DATA_X_H uint8
	REG_DATA_X_L uint8
	REG_DATA_Y_H uint8
	REG_DATA_Y_L uint8
	REG_DATA_Z_H uint8
	REG_DATA_Z_L uint8

	IMUInterface
}

type IMUInterface interface {
	XRead() int16
	YRead() int16
	ZRead() int16
	ReadSample() (int16, int16, int16)
	ReadSampleInDegrees() (float64, float64, float64)
}

func ReadAxis(dev *i2c.I2C, highReg, lowReg uint8, offset int16) int16 {
	var (
		h    uint8 = 0
		l    uint8 = 0
		hReg uint8 = 0
		lReg uint8 = 0
		err  error
	)

	if h, err = dev.ReadRegU8(hReg); err != nil {
		return 0
	}
	if l, err = dev.ReadRegU8(lReg); err != nil {
		return 0
	}

	val := int32(uint16(h<<8)|uint16(l)) - int32(offset)
	if val > 32768 {
		val -= 65536
	}
	return int16(val)
}
