package mpu9250

import (
	"fmt"
	"time"

	"github.com/gurupras/quadcopter"
	log "github.com/sirupsen/logrus"
)

const (
	MPU6500_ADDRESS          = 0x68
	REG_POWER_MANAGEMENT_1   = 0x6B
	REG_POWER_MANAGEMENT_2   = 0x6C
	REG_SAMPLE_RATE_DIV      = 0x19
	REG_CONFIG               = 0x1A
	REG_GYROSCOPE_CONFIG     = 0x1B
	REG_ACCELEROMETER_CONFIG = 0x1C
	REG_INTERRUPT_ENABLE     = 0x38
	REG_FIFO_ENABLE          = 0x23
	REG_I2C_MASTER_CONTROL   = 0x24
	REG_USER_CONTROL         = 0x6A

	REG_FIFO_COUNT_H = 0x72
	REG_FIFO_COUNT_L = 0x73
	REG_FIFO_RW      = 0x74

	REG_ACCELEROMETER_DATA_X_H = 0x3B
	REG_ACCELEROMETER_DATA_X_L = 0x3C
	REG_ACCELEROMETER_DATA_Y_H = 0x3D
	REG_ACCELEROMETER_DATA_Y_L = 0x3E
	REG_ACCELEROMETER_DATA_Z_H = 0x3F
	REG_ACCELEROMETER_DATA_Z_L = 0x40

	REG_ACCELEROMETER_OFFSET_X_H = 0x77
	REG_ACCELEROMETER_OFFSET_X_L = 0x78
	REG_ACCELEROMETER_OFFSET_Y_H = 0x7A
	REG_ACCELEROMETER_OFFSET_Y_L = 0x7B
	REG_ACCELEROMETER_OFFSET_Z_H = 0x7D
	REG_ACCELEROMETER_OFFSET_Z_L = 0x7E

	REG_TEMPERATURE_DATA_H = 0x41
	REG_TEMPERATURE_DATA_L = 0x42

	REG_GYROSCOPE_DATA_X_H = 0x43
	REG_GYROSCOPE_DATA_X_L = 0x44
	REG_GYROSCOPE_DATA_Y_H = 0x45
	REG_GYROSCOPE_DATA_Y_L = 0x46
	REG_GYROSCOPE_DATA_Z_H = 0x47
	REG_GYROSCOPE_DATA_Z_L = 0x48

	REG_GYROSCOPE_OFFSET_X_H = 0x13
	REG_GYROSCOPE_OFFSET_X_L = 0x14
	REG_GYROSCOPE_OFFSET_Y_H = 0x15
	REG_GYROSCOPE_OFFSET_Y_L = 0x16
	REG_GYROSCOPE_OFFSET_Z_H = 0x17
	REG_GYROSCOPE_OFFSET_Z_L = 0x18

	AK8963_ADDRESS           = 0x0C
	REG_MAGNETOMETER_ST1     = 0x02
	REG_MAGNETOMETER_ST2     = 0x09
	REG_MAGNETOMETER_CONTROL = 0x0A

	REG_MAGNETOMETER_DATA_X_L = 0x03
	REG_MAGNETOMETER_DATA_X_H = 0x04
	REG_MAGNETOMETER_DATA_Y_L = 0x05
	REG_MAGNETOMETER_DATA_Y_H = 0x06
	REG_MAGNETOMETER_DATA_Z_L = 0x07
	REG_MAGNETOMETER_DATA_Z_H = 0x08
)

type MPU9250 struct {
	mpu6500     *quadcopter.I2CDevice
	ak8963      *quadcopter.I2CDevice
	Weight      float64 // Weight applied to accelerometer
	initialized bool
}

func New(bus int) *MPU9250 {
	ret := &MPU9250{
		mpu6500: quadcopter.NewI2CDevice(MPU6500_ADDRESS, bus),
		ak8963:  quadcopter.NewI2CDevice(AK8963_ADDRESS, bus),
	}
	return ret
}

func (m *MPU9250) Init() error {
	if m.initialized {
		return nil
	}
	var err error

	if err = m.mpu6500.SetAsCurrentDevice(); err != nil {
		return fmt.Errorf("Failed to set current device to mpu6500: %v", err)
	}

	err = m.Calibrate()
	if err != nil {
		return fmt.Errorf("Failed to calibrate device: %v", err)
	}

	delay := 10 * time.Millisecond

	// Set sample rate (max 200HZ)
	if err = m.mpu6500.WriteRegU8(REG_SAMPLE_RATE_DIV, 0x04); err != nil {
		return fmt.Errorf("Failed to set sample rate: %v", err)
	}
	time.Sleep(delay)

	// Reset all sensors
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_1, 0x00); err != nil {
		return fmt.Errorf("Failed to reset all sensors: %v", err)
	}
	time.Sleep(delay)

	// Power management
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_1, 0x01); err != nil {
		return fmt.Errorf("Failed to reset all sensors: %v", err)
	}
	time.Sleep(delay)

	// Config
	if err = m.mpu6500.WriteRegU8(REG_CONFIG, 0x00); err != nil {
		return fmt.Errorf("Failed to configure IMU: %v", err)
	}
	time.Sleep(delay)

	// Gyroscope config
	if err = m.mpu6500.WriteRegU8(REG_GYROSCOPE_CONFIG, 0b00000000); err != nil {
		return fmt.Errorf("Failed to configure gyroscope: %v", err)
	}
	time.Sleep(delay)

	// Accelerometer config
	if err = m.mpu6500.WriteRegU8(REG_ACCELEROMETER_CONFIG, 0b0000000); err != nil {
		return fmt.Errorf("Failed to configure accelerometer: %v", err)
	}
	time.Sleep(delay)

	// Interrupt register
	if err = m.mpu6500.WriteRegU8(REG_INTERRUPT_ENABLE, 0x1); err != nil {
		return fmt.Errorf("Failed to enable interrupt (raw data ready): %v", err)
	}
	time.Sleep(delay)

	if err = m.ak8963.SetAsCurrentDevice(); err != nil {
		return fmt.Errorf("Failed to set current device to ak8963: %v", err)
	}
	time.Sleep(delay)

	mode := uint8((0b0001 << 4) + 0b0110)
	// Mode
	if err = m.ak8963.WriteRegU8(REG_MAGNETOMETER_CONTROL, mode); err != nil {
		return fmt.Errorf("Failed to set magnetometer mode: %v", err)
	}
	time.Sleep(delay)
	return nil
}

func (m *MPU9250) Calibrate() error {
	var err error
	delay := 100 * time.Millisecond

	// Reset device
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_1, 0x80); err != nil {
		return fmt.Errorf("Failed to reset all sensors: %v", err)
	}
	time.Sleep(delay)

	// Configure clock source
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_1, 0x01); err != nil {
		return fmt.Errorf("Failed to configure clock source to be PLL gyroscope reference: %v", err)
	}
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_2, 0x00); err != nil {
		return fmt.Errorf("Failed to configure clock source to use internal oscillator: %v", err)
	}
	time.Sleep(delay)

	// Configure device for calibration

	// Disable interrupts
	if err = m.mpu6500.WriteRegU8(REG_INTERRUPT_ENABLE, 0x00); err != nil {
		return fmt.Errorf("Failed to disable interrupts: %v", err)
	}
	// Disable FIFO
	if err = m.mpu6500.WriteRegU8(REG_FIFO_ENABLE, 0x00); err != nil {
		return fmt.Errorf("Failed to disable FIFO: %v", err)
	}
	// Turn on internal clock
	if err = m.mpu6500.WriteRegU8(REG_POWER_MANAGEMENT_1, 0x00); err != nil {
		return fmt.Errorf("Failed to turn on internal oscillator: %v", err)
	}

	// Disable I2C master
	if err = m.mpu6500.WriteRegU8(REG_I2C_MASTER_CONTROL, 0x00); err != nil {
		return fmt.Errorf("Failed to disable I2C master control: %v", err)
	}

	// Disable FIFO and I2C master modes
	if err = m.mpu6500.WriteRegU8(REG_USER_CONTROL, 0x00); err != nil {
		return fmt.Errorf("Failed to disable user control: %v", err)
	}

	// Reset FIFO and DMP
	if err = m.mpu6500.WriteRegU8(REG_USER_CONTROL, 0x0C); err != nil {
		return fmt.Errorf("Failed to reset all sensors: %v", err)
	}
	time.Sleep(15 * time.Microsecond)

	// Configure MPU6500 for calibration

	// Set LPF to 188HZ
	if err = m.mpu6500.WriteRegU8(REG_CONFIG, 0x01); err != nil {
		return fmt.Errorf("Failed to reset all sensors: %v", err)
	}

	// Set sample rate
	if err = m.mpu6500.WriteRegU8(REG_SAMPLE_RATE_DIV, 0x00); err != nil {
		return fmt.Errorf("Failed to set sample rate: %v", err)
	}
	time.Sleep(delay)

	// Gyroscope config
	if err = m.mpu6500.WriteRegU8(REG_GYROSCOPE_CONFIG, 0x00); err != nil {
		return fmt.Errorf("Failed to configure gyroscope: %v", err)
	}
	time.Sleep(delay)

	// Accelerometer config
	if err = m.mpu6500.WriteRegU8(REG_ACCELEROMETER_CONFIG, 0x00); err != nil {
		return fmt.Errorf("Failed to configure accelerometer: %v", err)
	}
	time.Sleep(delay)

	// Configure FIFO to capture accelerometer and gyroscope for bias calcuation
	if err = m.mpu6500.WriteRegU8(REG_USER_CONTROL, 0x40); err != nil {
		return fmt.Errorf("Failed to enable FIFO: %v", err)
	}

	if err = m.mpu6500.WriteRegU8(REG_FIFO_ENABLE, 0x78); err != nil {
		return fmt.Errorf("Failed to enable gyroscope and accelerometer for FIFO: %v", err)
	}
	time.Sleep(40 * time.Millisecond) // accumulate 40 samples = 480bytes. max available is 512 bytes

	// Turn off FIFO
	if err = m.mpu6500.WriteRegU8(REG_FIFO_ENABLE, 0x00); err != nil {
		return fmt.Errorf("Failed to disable FIFO: %v", err)
	}

	// Read FIFO sample count
	fifoCount := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_COUNT_H, REG_FIFO_COUNT_L, 0)
	packetCount := int(fifoCount) / 12 // Number of sets of full gyroscope and accelerometer data

	accBias := &quadcopter.Vec3{}
	gyroBias := &quadcopter.Vec3{}

	for i := 0; i < packetCount; i++ {
		aX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO accelerometer X: %v", err)
		}
		aY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO accelerometer Y: %v", err)
		}
		aZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO accelerometer Z: %v", err)
		}
		gX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO gyroscope X: %v", err)
		}
		gY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO gyroscope Y: %v", err)
		}
		gZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_FIFO_RW, REG_FIFO_RW, 0)
		if err != nil {
			return fmt.Errorf("Failed to read FIFO gyroscope Z: %v", err)
		}
		accBias.X += float64(aX)
		accBias.Y += float64(aY)
		accBias.Z += float64(aZ)

		gyroBias.X += float64(gX)
		gyroBias.Y += float64(gY)
		gyroBias.Z += float64(gZ)
	}

	accBias.Divide(float64(packetCount))
	gyroBias.Divide(float64(packetCount))

	accBiasX := int32(accBias.X)
	accBiasY := int32(accBias.Y)
	accBiasZ := int32(accBias.Z)

	if accBiasZ > 0 {
		accBiasZ -= 16384
	} else {
		accBiasZ += 16384
	}

	writeGyroOffset := func(v int16, h, l byte) error {
		msb := uint8(-v >> 8)
		lsb := uint8(-v & 0x00FF)
		// Divide by 4 to get 32.9 LSB/deg/s
		msb /= 4
		lsb /= 4

		if err := m.mpu6500.I2C.WriteRegU8(h, msb); err != nil {
			return fmt.Errorf("Failed to write register: %X: %v\n", h, err)
		}
		if err := m.mpu6500.I2C.WriteRegU8(l, lsb); err != nil {
			return fmt.Errorf("Failed to write to register: %X: %v\n", l, err)
		}
		return nil
	}
	log.Debugf("Raw calculated gyroscope bias: X=%v Y=%v Z=%v", gyroBias.X, gyroBias.Y, gyroBias.Z)
	gyroBiasX := int16(gyroBias.X)
	gyroBiasY := int16(gyroBias.Y)
	gyroBiasZ := int16(gyroBias.Z)

	{
		factoryBiasX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_OFFSET_X_H, REG_GYROSCOPE_OFFSET_X_L, 0)
		factoryBiasY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_OFFSET_Y_H, REG_GYROSCOPE_OFFSET_Y_L, 0)
		factoryBiasZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_OFFSET_Z_H, REG_GYROSCOPE_OFFSET_Z_L, 0)
		log.Debugf("Gyroscope factory bias: X=%v  Y=%v  Z=%v\n", factoryBiasX, factoryBiasY, factoryBiasZ)
	}

	// Update Gyroscope bias
	if err := writeGyroOffset(gyroBiasX, REG_GYROSCOPE_OFFSET_X_H, REG_GYROSCOPE_OFFSET_X_L); err != nil {
		return fmt.Errorf("Failed to write gyroscope X offset: %v", err)
	}
	if err := writeGyroOffset(gyroBiasY, REG_GYROSCOPE_OFFSET_Y_H, REG_GYROSCOPE_OFFSET_Y_L); err != nil {
		return fmt.Errorf("Failed to write gyroscope Y offset: %v", err)
	}
	if err := writeGyroOffset(gyroBiasZ, REG_GYROSCOPE_OFFSET_Z_H, REG_GYROSCOPE_OFFSET_Z_L); err != nil {
		return fmt.Errorf("Failed to write gyroscope Z offset: %v", err)
	}
	log.Debugf("Gyroscope calibrated bias: X=%v  Y=%v  Z=%v\n", gyroBiasX, gyroBiasY, gyroBiasZ)

	// Construct the accelerometer biases for push to the hardware accelerometer
	// bias registers. These registers contain factory trim values which must be
	// added to the calculated accelerometer biases; on boot up these registers
	// will hold non-zero values. In addition, bit 0 of the lower byte must be
	// preserved since it is used for temperature compensation calculations.
	// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	factoryBiasX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_OFFSET_X_H, REG_ACCELEROMETER_OFFSET_X_L, 0)
	factoryBiasY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_OFFSET_Y_H, REG_ACCELEROMETER_OFFSET_Y_L, 0)
	factoryBiasZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_OFFSET_Z_H, REG_ACCELEROMETER_OFFSET_Z_L, 0)

	log.Debugf("Accelerometer factory bias: X=%v  Y=%v  Z=%v\n", factoryBiasX, factoryBiasY, factoryBiasZ)

	// Bit 0 of the lower byte must be preserved since it is used for temperature compensation calculation
	maskX := factoryBiasX & 0x1
	maskY := factoryBiasY & 0x1
	maskZ := factoryBiasZ & 0x1

	// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	factoryBiasX -= int16(accBiasX >> 3)
	factoryBiasY -= int16(accBiasY >> 3)
	factoryBiasZ -= int16(accBiasZ >> 3)

	factoryBiasX |= maskX
	factoryBiasY |= maskY
	factoryBiasZ |= maskZ

	log.Debugf("Accelerometer calibrated bias: X=%v  Y=%v  Z=%v\n", factoryBiasX, factoryBiasY, factoryBiasZ)

	writeOffset := func(v int16, h, l byte) error {
		msb := uint8(v >> 8)
		lsb := uint8(v & 0x00FF)

		if err := m.mpu6500.I2C.WriteRegU8(h, msb); err != nil {
			return fmt.Errorf("Failed to write register: %X: %v\n", h, err)
		}
		if err := m.mpu6500.I2C.WriteRegU8(l, lsb); err != nil {
			return fmt.Errorf("Failed to write to register: %X: %v\n", l, err)
		}
		return nil
	}

	if err := writeOffset(factoryBiasX, REG_ACCELEROMETER_OFFSET_X_H, REG_ACCELEROMETER_OFFSET_X_L); err != nil {
		return fmt.Errorf("Failed to set accelerometer X offset: %v", err)
	}
	if err := writeOffset(factoryBiasY, REG_ACCELEROMETER_OFFSET_Y_H, REG_ACCELEROMETER_OFFSET_Y_L); err != nil {
		return fmt.Errorf("Failed to set accelerometer Y offset: %v", err)
	}
	if err := writeOffset(factoryBiasZ, REG_ACCELEROMETER_OFFSET_Z_H, REG_ACCELEROMETER_OFFSET_Z_L); err != nil {
		return fmt.Errorf("Failed to set accelerometer Z offset: %v", err)
	}
	return nil
}

func (m *MPU9250) ReadRawAccelerometer() quadcopter.Vec3 {
	accX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_X_H, REG_ACCELEROMETER_DATA_X_L, 0)
	accY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_Y_H, REG_ACCELEROMETER_DATA_Y_L, 0)
	accZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_Z_H, REG_ACCELEROMETER_DATA_Z_L, 0)

	return quadcopter.Vec3{
		X: float64(accX),
		Y: float64(accY),
		Z: float64(accZ),
	}
}

func (m *MPU9250) ReadRawGyroscope() quadcopter.Vec3 {
	gyroX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_X_H, REG_GYROSCOPE_DATA_X_L, 0)
	gyroY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_Y_H, REG_GYROSCOPE_DATA_Y_L, 0)
	gyroZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_Z_H, REG_GYROSCOPE_DATA_Z_L, 0)

	return quadcopter.Vec3{
		X: float64(gyroX),
		Y: float64(gyroY),
		Z: float64(gyroZ),
	}
}

func (m *MPU9250) ReadRawMagnetometer() quadcopter.Vec3 {
	magX := quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_X_H, REG_MAGNETOMETER_DATA_X_L, 0)
	magY := quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_Y_H, REG_MAGNETOMETER_DATA_Y_L, 0)
	magZ := quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_Z_H, REG_MAGNETOMETER_DATA_Z_L, 0)

	return quadcopter.Vec3{
		X: float64(magX),
		Y: float64(magY),
		Z: float64(magZ),
	}
}

func (m *MPU9250) ReadSample() (quadcopter.Vec3, quadcopter.Vec3, quadcopter.Vec3) {
	m.mpu6500.SetAsCurrentDevice()

	acc := m.ReadRawAccelerometer()
	gyro := m.ReadRawGyroscope()

	m.ak8963.SetAsCurrentDevice()

	var mag quadcopter.Vec3
	for {
		mag = m.ReadRawMagnetometer()
		if b, err := m.ak8963.ReadRegU8(REG_MAGNETOMETER_ST2); err != nil {
			log.Warnf("Encountered error reading ST2 register: %v\n", err)
			break
		} else {
			if b == 0b10000 {
				break
			}
		}
	}

	mag.X = float64(mag.X) * 4912 / 32760
	mag.Y = float64(mag.Y) * 4912 / 32760
	mag.Z = float64(mag.Z) * 4912 / 32760

	acc.X = float64(acc.X) * 2 / 32768
	acc.Y = float64(acc.Y) * 2 / 32768
	acc.Z = float64(acc.Z) * 2 / 32768

	gyro.X = float64(gyro.X) * 250 / 32768
	gyro.Y = float64(gyro.Y) * 250 / 32768
	gyro.Z = float64(gyro.Z) * 250 / 32768

	return acc, gyro, mag
}

func round(val float64) string {
	return fmt.Sprintf("%.2f", val)
}
