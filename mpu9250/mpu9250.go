package mpu9250

import (
	"fmt"
	"math"
	"time"

	"github.com/gurupras/quadcopter"
	log "github.com/sirupsen/logrus"
)

const (
	MPU6500_ADDRESS          = 0x68
	REG_POWER_MANAGEMENT_1   = 0x6B
	REG_SAMPLE_RATE_DIV      = 0x19
	REG_CONFIG               = 0x1A
	REG_GYROSCOPE_CONFIG     = 0x1B
	REG_ACCELEROMETER_CONFIG = 0x1C
	REG_INTERRUPT_ENABLE     = 0x38

	REG_ACCELEROMETER_DATA_X_H = 0x3B
	REG_ACCELEROMETER_DATA_X_L = 0x3C
	REG_ACCELEROMETER_DATA_Y_H = 0x3D
	REG_ACCELEROMETER_DATA_Y_L = 0x3E
	REG_ACCELEROMETER_DATA_Z_H = 0x3F
	REG_ACCELEROMETER_DATA_Z_L = 0x40

	REG_TEMPERATURE_DATA_H = 0x41
	REG_TEMPERATURE_DATA_L = 0x42

	REG_GYROSCOPE_DATA_X_H = 0x43
	REG_GYROSCOPE_DATA_X_L = 0x44
	REG_GYROSCOPE_DATA_Y_H = 0x45
	REG_GYROSCOPE_DATA_Y_L = 0x46
	REG_GYROSCOPE_DATA_Z_H = 0x47
	REG_GYROSCOPE_DATA_Z_L = 0x48

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

	delay := 10 * time.Millisecond

	if err = m.mpu6500.SetAsCurrentDevice(); err != nil {
		return fmt.Errorf("Failed to set current device to mpu6500: %v", err)
	}
	// Set sample rate
	if err = m.mpu6500.WriteRegU8(REG_SAMPLE_RATE_DIV, 0x00); err != nil {
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
	if err = m.mpu6500.WriteRegU8(REG_ACCELEROMETER_CONFIG, 0b00000000); err != nil {
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

func (m *MPU9250) ReadSample() (int16, int16, int16) {
	m.mpu6500.SetAsCurrentDevice()

	accX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_X_H, REG_ACCELEROMETER_DATA_X_L, 0)
	accY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_Y_H, REG_ACCELEROMETER_DATA_Y_L, 0)
	accZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_ACCELEROMETER_DATA_Z_H, REG_ACCELEROMETER_DATA_Z_L, 0)

	gyroX := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_X_H, REG_GYROSCOPE_DATA_X_L, 0)
	gyroY := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_Y_H, REG_GYROSCOPE_DATA_Y_L, 0)
	gyroZ := quadcopter.ReadAxis(m.mpu6500.I2C, REG_GYROSCOPE_DATA_Z_H, REG_GYROSCOPE_DATA_Z_L, 0)

	m.ak8963.SetAsCurrentDevice()

	var (
		magX int16
		magY int16
		magZ int16
	)
	for {
		magX = quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_X_H, REG_MAGNETOMETER_DATA_X_L, 0)
		magY = quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_Y_H, REG_MAGNETOMETER_DATA_Y_L, 0)
		magZ = quadcopter.ReadAxis(m.ak8963.I2C, REG_MAGNETOMETER_DATA_Z_H, REG_MAGNETOMETER_DATA_Z_L, 0)

		if b, err := m.ak8963.ReadRegU8(REG_MAGNETOMETER_ST2); err != nil {
			log.Warnf("Encountered error reading ST2 register: %v\n", err)
			break
		} else {
			if b == 0b10000 {
				break
			}
		}
	}

	mX := (float64(magX) / math.Pow(2.0, 15.0)) * 4900.0
	mY := (float64(magY) / math.Pow(2.0, 15.0)) * 4900.0
	mZ := (float64(magZ) / math.Pow(2.0, 15.0)) * 4900.0

	aX := (float64(accX) / math.Pow(2.0, 15.0)) * 2.0
	aY := (float64(accY) / math.Pow(2.0, 15.0)) * 2.0
	aZ := (float64(accZ) / math.Pow(2.0, 15.0)) * 2.0

	gX := (float64(gyroX) / math.Pow(2.0, 15.0)) * 250.0
	gY := (float64(gyroY) / math.Pow(2.0, 15.0)) * 250.0
	gZ := (float64(gyroZ) / math.Pow(2.0, 15.0)) * 250.0

	log.Debugf("Accelerometer: x=%v y=%v z=%v\n", aX, aY, aZ)
	log.Debugf("Gyroscope:     x=%v y=%v z=%v\n", gX, gY, gZ)
	log.Debugf("Magnetometer:  x=%v y=%v z=%v\n", mX, mY, mZ)

	return 0, 0, 0
}
