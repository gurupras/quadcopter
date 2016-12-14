package quadcopter

type SensorFusion struct {
	*Itg3200
	*Adxl345
	IMU
	Weight float64 // Weight applied to accelerometer
}

func NewSensorFusion(itg *Itg3200, adxl345 *Adxl345) *SensorFusion {
	sf := new(SensorFusion)
	sf.Itg3200 = itg
	sf.Adxl345 = adxl345
	sf.Weight = 0.02
	return sf
}

func (sf *SensorFusion) ReadSample() (int16, int16, int16) {
	return 0, 0, 0 //Unimplemented
}

func (sf *SensorFusion) ReadSampleInDegrees() (float64, float64, float64) {
	xGyro, yGyro, zGyro := sf.Itg3200.ReadSampleInDegrees()
	xAcc, yAcc, zAcc := sf.Adxl345.ReadSampleInDegrees()

	x := (xGyro * (1.0 - sf.Weight)) + (xAcc * sf.Weight)
	y := (yGyro * (1.0 - sf.Weight)) + (yAcc * sf.Weight)
	z := (zGyro * (1.0 - sf.Weight)) + (zAcc * sf.Weight)
	return x, y, z
}
