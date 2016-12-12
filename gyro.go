package quadcopter

type Gyroscope struct {
	XOffset int16
	YOffset int16
	ZOffset int16

	GyroscopeInterface
}

type GyroscopeInterface interface {
	XRead() int16
	YRead() int16
	ZRead() int16
}
