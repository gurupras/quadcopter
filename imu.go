package quadcopter

type IMU struct {
	XOffset int16
	YOffset int16
	ZOffset int16

	IMUInterface
}

type IMUInterface interface {
	XRead() int16
	YRead() int16
	ZRead() int16
}
