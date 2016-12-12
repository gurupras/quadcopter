package quadcopter

import "time"
import "fmt"

const (
	ITG3200_ADDR        uint8   = 0x68 // 7-bit address
	ITG3200_SENSITIVITY float64 = 14.375

	// Registers
	REG_WHO_AM_I    uint8 = 0x00
	REG_SMPLRT_DIV  uint8 = 0x15
	REG_DLPF_FS     uint8 = 0x16
	REG_INT_CFG     uint8 = 0x17
	REG_INT_STATUS  uint8 = 0x1A
	REG_TEMP_OUT_H  uint8 = 0x1B
	REG_TEMP_OUT_L  uint8 = 0x1C
	REG_GYRO_XOUT_H uint8 = 0x1D
	REG_GYRO_XOUT_L uint8 = 0x1E
	REG_GYRO_YOUT_H uint8 = 0x1F
	REG_GYRO_YOUT_L uint8 = 0x20
	REG_GYRO_ZOUT_H uint8 = 0x21
	REG_GYRO_ZOUT_L uint8 = 0x22
	REG_PWRMGM      uint8 = 0x3E

	// LPF Bandwidths
	DLPF_FS_SEL uint8 = (3 << 3)
	LPFBW_256HZ uint8 = 0x00
	LPFBW_188HZ uint8 = 0x01
	LPFBW_98HZ  uint8 = 0x02
	LPFBW_42HZ  uint8 = 0x03
	LPFBW_20HZ  uint8 = 0x04
	LPFBW_10HZ  uint8 = 0x05
	LPFBW_5HZ   uint8 = 0x06
)

type Itg3200 struct {
	Gyroscope
	*I2CDevice
	initialized bool
}

func NewItg3200(dev *I2CDevice) *Itg3200 {
	itg := new(Itg3200)
	itg.I2CDevice = dev
	itg.initialized = false
	return itg
}

func (itg *Itg3200) Init() {
	if itg.initialized {
		return
	}
	itg.SetAsCurrentDevice()
	itg.WriteRegU8(REG_PWRMGM, 0x0)
	itg.WriteRegU8(REG_SMPLRT_DIV, 0xa)
	itg.WriteRegU8(REG_DLPF_FS, uint8(DLPF_FS_SEL|LPFBW_256HZ))
	itg.WriteRegU8(REG_INT_CFG, 0x0)
	itg.initialized = true
}

func (itg *Itg3200) Calibrate(loopCount int, sleepPeriod time.Duration) {
	itg.Init()

	var (
		x_tmp int64 = 0
		y_tmp int64 = 0
		z_tmp int64 = 0
	)

	fmt.Println("Calibrating ITG-3200 ...")
	for i := 0; i < loopCount; i++ {
		x_tmp += int64(itg.XRead())
		y_tmp += int64(itg.YRead())
		z_tmp += int64(itg.ZRead())
		time.Sleep(sleepPeriod)
	}
	fmt.Println("Finished calibration")

	itg.XOffset = int16(x_tmp / int64(loopCount))
	itg.YOffset = int16(y_tmp / int64(loopCount))
	itg.ZOffset = int16(z_tmp / int64(loopCount))
}

func (itg *Itg3200) Stop() {
	itg.WriteRegU8(REG_PWRMGM, 0x20)
}

func (itg *Itg3200) TempAdcToC(value uint16) float64 {
	return 35.0 + (float64((int(value) + 13200)) / 280.0)
}

func (itg *Itg3200) XRead() int16 {
	return itg.ReadAxis(AXIS_X)
}
func (itg *Itg3200) YRead() int16 {
	return itg.ReadAxis(AXIS_Y)
}

func (itg *Itg3200) ZRead() int16 {
	return itg.ReadAxis(AXIS_Z)
}

func (itg *Itg3200) ReadAxis(axis Axis) int16 {
	var (
		h      uint8 = 0
		l      uint8 = 0
		hReg   uint8 = 0
		lReg   uint8 = 0
		offset int16 = 0
		err    error
	)

	switch axis {
	case AXIS_X:
		hReg = REG_GYRO_XOUT_H
		lReg = REG_GYRO_XOUT_L
		offset = itg.XOffset
	case AXIS_Y:
		hReg = REG_GYRO_YOUT_H
		lReg = REG_GYRO_YOUT_L
		offset = itg.YOffset
	case AXIS_Z:
		hReg = REG_GYRO_ZOUT_H
		lReg = REG_GYRO_ZOUT_L
		offset = itg.ZOffset
	}

	if h, err = itg.ReadRegU8(hReg); err != nil {
		return 0
	}
	if l, err = itg.ReadRegU8(lReg); err != nil {
		return 0
	}

	val := int16(uint16(h<<8)|uint16(l)) - offset
	return val
}

func (itg *Itg3200) AdcToAngle(value int16) float64 {
	return float64(value) / ITG3200_SENSITIVITY
}

func (itg *Itg3200) ReadSample() (x int16, y int16, z int16) {
	x = itg.ReadAxis(AXIS_X)
	y = itg.ReadAxis(AXIS_Y)
	z = itg.ReadAxis(AXIS_Z)
	return
}

func (itg *Itg3200) ReadSampleInDegrees() (xDeg float64, yDeg float64, zDeg float64) {
	x, y, z := itg.ReadSample()

	xDeg = itg.AdcToAngle(x)
	yDeg = itg.AdcToAngle(y)
	zDeg = itg.AdcToAngle(z)
	return
}
