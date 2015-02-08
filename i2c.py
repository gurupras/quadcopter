import threading
import sys

import numpy

from fcntl import ioctl

import smbus

class I2cDevice(object):
        I2C_SLAVE = 0x0703
	lock = threading.Lock()
	sleep_period = 0.05

	def __init__(self, i2c_fd, addr):
		self.i2c_fd = i2c_fd.fileno()
		self.addr   = addr

	def set_i2c_device(self):
		if ioctl(self.i2c_fd, I2cDevice.I2C_SLAVE, self.addr) < 0:
			print 'Failed to set %s as I2C device' % (self.name)
			sys.exit(-1)

	def i2c_read_register(self, register):
		return numpy.uint16(smbus.smbus_read_byte_data(self.i2c_fd, register))

	def i2c_write_register(self, register, value):
		if isinstance(value, numpy.uint16):
			value = numpy.asscalar(value)
		return smbus.smbus_write_byte_data(self.i2c_fd, register, value)

	def i2c_read(self):
		return numpy.uint16(smbus.smbus_read_byte(self.i2c_fd))

	def i2c_write(self, value):
		return smbus.smbus_write_byte(self.i2c_fd, value)

