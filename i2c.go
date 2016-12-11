package quadcopter

import (
	"fmt"
	"os"
	"sync"
	"syscall"

	"github.com/gurupras/go-i2c"
)

const (
	I2C_SLAVE uintptr = 0x0703
)

var (
	I2CMutex sync.Mutex
)

type I2CDevice struct {
	*i2c.I2C
	fd   uintptr
	Addr uintptr
}

func NewI2CDevice(addr uint8, bus int) *I2CDevice {
	dev := new(I2CDevice)
	i2c, fd, err := newI2C(addr, bus)
	if err != nil {
		return nil
	}
	dev.I2C = i2c
	dev.fd = fd
	dev.Addr = uintptr(addr)
	return dev
}

func (dev *I2CDevice) SetAsCurrentDevice() error {
	I2CMutex.Lock()
	err := ioctl(dev.fd, I2C_SLAVE, dev.Addr)
	I2CMutex.Unlock()
	return err
}

func newI2C(addr uint8, bus int) (*i2c.I2C, uintptr, error) {
	f, err := os.OpenFile(fmt.Sprintf("/dev/i2c-%d", bus), os.O_RDWR, 0600)
	if err != nil {
		return nil, 0, err
	}
	if err := ioctl(f.Fd(), I2C_SLAVE, uintptr(addr)); err != nil {
		return nil, 0, err
	}
	this := &i2c.I2C{f}
	return this, f.Fd(), nil
}
func ioctl(fd, cmd, arg uintptr) error {
	_, _, err := syscall.Syscall6(syscall.SYS_IOCTL, fd, cmd, arg, 0, 0, 0)
	if err != 0 {
		return err
	}
	return nil
}
