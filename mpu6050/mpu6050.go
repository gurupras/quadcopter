package mpu6050

import (
	"github.com/gurupras/quadcopter"
)

type MPU6050 struct {
	*Itg3200
	*Adxl345
}

func New(itg3200 *Itg3200, adxl345 *Adxl345) *MPU6050 {
	ret := &MPU6050{
		Itg3200: itg3200,
		Adxl345: adxl345,
	}
	return ret
}

func (m *MPU6050) ReadSample() (quadcopter.Vec3, quadcopter.Vec3) {
	acc := m.Adxl345.ReadSample()
	gyro := m.Itg3200.ReadSample()
	return acc, gyro
}
