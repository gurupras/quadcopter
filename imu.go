package quadcopter

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

	IMUInterface
}

type IMUInterface interface {
	XRead() int16
	YRead() int16
	ZRead() int16
	ReadSample() (int16, int16, int16)
	ReadSampleInDegrees() (float64, float64, float64)
}
