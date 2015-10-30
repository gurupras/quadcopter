import sys,os,re
from i2c import I2cDevice

class ESC(I2cDevice):
	def __init__(self, delay, i2c_fd, addr):
		super(ESC, self).__init__(i2c_fd, addr)
		self.delay = delay
		self.speed = 0


