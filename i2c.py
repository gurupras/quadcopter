import threading
import sys

from fcntl import ioctl

class I2cDevice(object):
	I2C_SLAVE = 0x0703

	lock = threading.lock()

	def __init__(self, i2c_fd, addr):
		self.i2c_fd = i2c_fd
		self.addr   = addr

	def set_i2c_device(self):
		if ioctl(self.i2c_fd, I2cDevice.I2C_SLAVE, self.addr) < 0:
			print 'Failed to set %s as I2C device' % (self.name)
			sys.exit(-1)

	def i2c_read_register(self, register):
		return self.i2c_smbus_read_byte_data(register)

	def i2c_write_register(self, register, value):
		self.i2c_smbus_write_byte_data(register, value)
