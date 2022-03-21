package mpu6050

import (
	"fmt"
	"math"
	"time"

	"github.com/gurupras/quadcopter"
)

const (
	ADXL345_ADDRESS uint8 = 0x53 //I2C address of ADXL345

	REG_DEVID                   uint8 = 0x00 //Device ID Register
	REG_THRESH_TAP              uint8 = 0x1D //Tap Threshold
	REG_OFFSET_X                uint8 = 0x1E //X-axis offset
	REG_OFFSET_Y                uint8 = 0x1F //Y-axis offset
	REG_OFFSET_Z                uint8 = 0x20 //Z-axis offset
	REG_TAP_DURATION            uint8 = 0x21 //Tap Duration
	REG_TAP_LATENCY             uint8 = 0x22 //Tap latency
	REG_TAP_WINDOW              uint8 = 0x23 //Tap window
	REG_THRESH_ACTIVITY         uint8 = 0x24 //Activity Threshold
	REG_THRESH_INACTIVITY       uint8 = 0x25 //Inactivity Threshold
	REG_TIME_INACTIVITY         uint8 = 0x26 //Inactivity Time
	REG_ACTIVITY_INACTIVITY_CTL uint8 = 0x27 //Axis enable control for activity and inactivity detection
	REG_THRESH_FREEFALL         uint8 = 0x28 //free-fall threshold
	REG_TIME_FREEFALL           uint8 = 0x29 //Free-Fall Time
	REG_TAP_AXES                uint8 = 0x2A //Axis control for tap/double tap
	REG_ACTIVITY_TAP_STATUS     uint8 = 0x2B //Source of tap/double tap
	REG_BW_RATE                 uint8 = 0x2C //Data rate and power mode control
	REG_POWER_CTL               uint8 = 0x2D //Power Control Register
	REG_INT_ENABLE              uint8 = 0x2E //Interrupt Enable Control
	REG_INT_MAP                 uint8 = 0x2F //Interrupt Mapping Control
	REG_INT_SOURCE              uint8 = 0x30 //Source of interrupts
	REG_DATA_FORMAT             uint8 = 0x31 //Data format control
	REG_DATA_X_L                uint8 = 0x32 //X-Axis Data 0
	REG_DATA_X_H                uint8 = 0x33 //X-Axis Data 1
	REG_DATA_Y_L                uint8 = 0x34 //Y-Axis Data 0
	REG_DATA_Y_H                uint8 = 0x35 //Y-Axis Data 1
	REG_DATA_Z_L                uint8 = 0x36 //Z-Axis Data 0
	REG_DATA_Z_H                uint8 = 0x37 //Z-Axis Data 1
	REG_FIFO_CTL                uint8 = 0x38 //FIFO control
	REG_FIFO_STATUS             uint8 = 0x39 //FIFO status

	ADXL345_RESOLUTION float64 = 0.004 //mg/lsb

	// Output Data Rates(Hz)
	BITS_RATE_3200HZ uint8 = 0x0F
	BITS_RATE_1600HZ uint8 = 0x0E
	BITS_RATE_800HZ  uint8 = 0x0D
	BITS_RATE_400HZ  uint8 = 0x0C
	BITS_RATE_200HZ  uint8 = 0x0B
	BITS_RATE_100HZ  uint8 = 0x0A

	// Power management bits
	BITS_PWR_LINK       uint8 = 1 << 5
	BITS_PWR_AUTO_SLEEP uint8 = 1 << 4
	BITS_PWR_MEASURE    uint8 = 1 << 3
	BITS_PWR_SLEEP      uint8 = 1 << 2
	BITS_PWR_WAKEUP_8Hz uint8 = 0x00
	BITS_PWR_WAKEUP_4Hz uint8 = 0x01
	BITS_PWR_WAKEUP_2Hz uint8 = 0x10
	BITS_PWR_WAKEUP_1Hz uint8 = 0x11

	// Data format bits
	BITS_DATA_FULL_RES  uint8 = 1 << 3
	BITS_DATA_RANGE_2G  uint8 = 0x00
	BITS_DATA_RANGE_4G  uint8 = 0x01
	BITS_DATA_RANGE_8G  uint8 = 0x10
	BITS_DATA_RANGE_16G uint8 = 0x11

	//Scaling

	// The base scale is the value for scaling under +/- 2g range.
	// This is also the default value under BITS_DATA_FULL_RES.
	// When BITS_DATA_FULL_RES is disabled, the scale changes based on the range specified.
	// +/- 4g  is BASE_SCALE * 2
	// +/- 8g  is BASE_SCALE * 4
	// +/- 16g is BASE_SCALE * 8
	BASE_SCALE float64 = 0.0039

	// FIFO_CTL bits
	FIFO_BYPASS  uint8 = 0x00
	FIFO_STORE   uint8 = 0x01
	FIFO_STREAM  uint8 = 0x10
	FIFO_TRIGGER uint8 = 0x11
)

type Adxl345 struct {
	*quadcopter.I2CDevice
	quadcopter.Accelerometer
	initialized bool
}

func NewAdxl345(dev *quadcopter.I2CDevice) *Adxl345 {
	adxl345 := new(Adxl345)
	adxl345.I2CDevice = dev
	adxl345.initialized = false
	return adxl345
}

func (adxl345 *Adxl345) Init() {
	if adxl345.initialized {
		return
	}
	adxl345.SetAsCurrentDevice()
	adxl345.WriteRegU8(REG_DATA_FORMAT, BITS_DATA_FULL_RES|BITS_DATA_RANGE_4G)
	adxl345.WriteRegU8(REG_FIFO_CTL, FIFO_STREAM)
	adxl345.WriteRegU8(REG_BW_RATE, BITS_RATE_100HZ)
	adxl345.WriteRegU8(REG_POWER_CTL, BITS_PWR_MEASURE)
	adxl345.initialized = true
}

func (adxl345 *Adxl345) Calibrate(loopCount int, sleepPeriod time.Duration) {
	adxl345.WriteRegU8(REG_POWER_CTL, 0x0)
	adxl345.WriteRegU8(REG_OFFSET_X, 0x0)
	adxl345.WriteRegU8(REG_OFFSET_Y, 0x0)
	adxl345.WriteRegU8(REG_OFFSET_Z, 0x0)

	adxl345.Init()

	var (
		x_tmp int64 = 0
		y_tmp int64 = 0
		z_tmp int64 = 0
	)

	fmt.Println("Calibrating ADXL-345 ...")
	for i := 0; i < loopCount; i++ {
		x_tmp += int64(adxl345.XRead())
		y_tmp += int64(adxl345.YRead())
		z_tmp += int64(adxl345.ZRead())

		time.Sleep(sleepPeriod)
	}
	fmt.Println("Finished calibration")

	x_offset := int16(0 - ((x_tmp / int64(loopCount)) / int64(4)))
	y_offset := int16(0 - ((y_tmp / int64(loopCount)) / int64(4)))
	z_offset := int16(0 - ((z_tmp / int64(loopCount)) / int64(4)))

	x_offset = int16((x_tmp / int64(loopCount)))
	y_offset = int16((y_tmp / int64(loopCount)))
	z_offset = int16((z_tmp / int64(loopCount)))

	// Write offsets
	adxl345.WriteRegU8(REG_POWER_CTL, 0x0)
	//adxl345.WriteRegS16BE(REG_OFFSET_X, x_offset)
	//adxl345.WriteRegS16BE(REG_OFFSET_Y, y_offset)
	//adxl345.WriteRegS16BE(REG_OFFSET_Z, z_offset)

	adxl345.XOffset = x_offset
	adxl345.YOffset = y_offset
	adxl345.ZOffset = z_offset

	// Restore measuring
	adxl345.WriteRegU8(REG_POWER_CTL, BITS_PWR_MEASURE)
}

func (adxl345 *Adxl345) Stop() {
	adxl345.WriteRegU8(REG_POWER_CTL, 0x0)
}

func (adxl345 *Adxl345) ReadAxis(axis quadcopter.Axis) int16 {
	var (
		hReg   uint8 = 0
		lReg   uint8 = 0
		offset int16 = 0
	)

	switch axis {
	case quadcopter.AXIS_X:
		hReg = REG_DATA_X_H
		lReg = REG_DATA_X_L
		offset = adxl345.XOffset
	case quadcopter.AXIS_Y:
		hReg = REG_DATA_Y_H
		lReg = REG_DATA_Y_L
		offset = adxl345.YOffset
	case quadcopter.AXIS_Z:
		hReg = REG_DATA_Z_H
		lReg = REG_DATA_Z_L
		offset = adxl345.ZOffset
	}
	return quadcopter.ReadAxis(adxl345.I2C, hReg, lReg, offset)
}

func (adxl345 *Adxl345) XRead() int16 {
	return adxl345.ReadAxis(quadcopter.AXIS_X)
}
func (adxl345 *Adxl345) YRead() int16 {
	return adxl345.ReadAxis(quadcopter.AXIS_Y)
}
func (adxl345 *Adxl345) ZRead() int16 {
	return adxl345.ReadAxis(quadcopter.AXIS_Z)
}

func (adxl345 *Adxl345) ReadSample() (int16, int16, int16) {
	return adxl345.ReadAxis(quadcopter.AXIS_X), adxl345.ReadAxis(quadcopter.AXIS_Y), adxl345.ReadAxis(quadcopter.AXIS_Z)
}

func (adxl345 *Adxl345) ReadSampleInG() (float64, float64, float64) {
	x, y, z := adxl345.ReadSample()
	return adxl345.AdcToG(x), adxl345.AdcToG(y), adxl345.AdcToG(z)
}

func (adxl345 *Adxl345) ReadSampleInDegrees() (float64, float64, float64) {
	x, y, z := adxl345.ReadSampleInG()
	return adxl345.GToDegrees(x, y, z)
}

func (adxl345 *Adxl345) AdcToG(value int16) float64 {
	return float64(value) * (BASE_SCALE * float64(BITS_DATA_RANGE_4G))
}

func (adxl345 *Adxl345) GToDegrees(xG, yG, zG float64) (float64, float64, float64) {
	xDeg := math.Atan2(xG, (math.Sqrt(math.Pow(yG, 2) + math.Pow(zG, 2))))
	yDeg := math.Atan2(yG, (math.Sqrt(math.Pow(xG, 2) + math.Pow(zG, 2))))
	//zDeg := math.Atan2(math.Sqrt(math.Pow(xG, 2)+math.Pow(yG, 2)) / zG)

	xDeg = (xDeg * 180.0) / 3.141592
	yDeg = (yDeg * 180.0) / 3.141592
	//zDeg = (zDeg * 180.0) / 3.141592

	return xDeg, yDeg, 0.0
}
