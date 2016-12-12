package quadcopter

type Gyroscope struct {
	XOffset uint16
	YOffset uint16
	ZOffset uint16

	GyroscopeInterface
}

type GyroscopeInterface interface {
	XRead() uint16
	YRead() uint16
	ZRead() uint16
}
