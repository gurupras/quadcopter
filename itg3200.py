import sys,os,time
import argparse
import numpy

from i2c import I2cDevice
from imu import Gyroscope


# TODO: Add temperature offsetting
class Itg3200(Gyroscope):
	ITG3200_ADDR	    = 0x68	# 7-bit address
	ITG3200_SENSITIVITY = 14.375

	# Registers
	REG_WHO_AM_I     = 0x00
	REG_SMPLRT_DIV   = 0x15
	REG_DLPF_FS      = 0x16
	REG_INT_CFG      = 0x17
	REG_INT_STATUS   = 0x1A
	REG_TEMP_OUT_H   = 0x1B
	REG_TEMP_OUT_L   = 0x1C
	REG_GYRO_XOUT_H  = 0x1D
	REG_GYRO_XOUT_L  = 0x1E
	REG_GYRO_YOUT_H  = 0x1F
	REG_GYRO_YOUT_L  = 0x20
	REG_GYRO_ZOUT_H  = 0x21
	REG_GYRO_ZOUT_L  = 0x22
	REG_PWRMGM       = 0x3E

	# LPF Bandwidths
	DLPF_FS_SEL      = (3 << 3)
	LPFBW_256HZ      = 0x00
	LPFBW_188HZ      = 0x01
	LPFBW_98HZ       = 0x02
	LPFBW_42HZ       = 0x03
	LPFBW_20HZ       = 0x04
	LPFBW_10HZ       = 0x05
	LPFBW_5HZ        = 0x06

	
	def __init__(self, i2c_fd, addr=ITG3200_ADDR):
		super(Itg3200, self).__init__(i2c_fd, addr)
		self.x_offset = 0x0
		self.y_offset = 0x0
		self.z_offset = 0x0

	def init(self):
		self.set_i2c_device()
		self.i2c_write_register(Itg3200.REG_PWRMGM, 0x0)
		self.i2c_write_register(Itg3200.REG_SMPLRT_DIV, 0x09)
		self.i2c_write_register(Itg3200.REG_DLPF_FS, Itg3200.DLPF_FS_SEL | Itg3200.LPFBW_188HZ)
		self.i2c_write_register(Itg3200.REG_INT_CFG, 0x0)

	def calibrate(self, loop=100, sleep_period=0.01):
		self.init()
		x_tmp, y_tmp, z_tmp = (0,) * 3

		for i in range(loop):
			x_tmp += self.read_x()
			y_tmp += self.read_y()
			z_tmp += self.read_z()
			time.sleep(sleep_period)

		self.x_offset = numpy.uint16(x_tmp / loop)
		self.y_offset = numpy.uint16(y_tmp / loop)
		self.z_offset = numpy.uint16(z_tmp / loop)

	def stop(self):
		self.i2c_write_register(Itg3200.REG_PWRMGM, 0x20)

	def temp_adc_to_c(self, value):
		return 35 + ((value + 13200) / 280.0)

	def adc_to_angle(self, value):
		return value / Itg3200.ITG3200_SENSITIVITY


	def gyro_read_axis(self, axis_h, axis_l, offset):
		h, l = (0, 0)
		h = self.i2c_read_register(axis_h)
		l = self.i2c_read_register(axis_l)
		return numpy.uint16(((h << 8) | l) - offset)

	def read_temp(self):
		return self.gyro_read_axis(Itg3200.REG_TEMP_OUT_H, Itg3200.REG_TEMP_OUT_L, self.temp_offset)

	def read_x(self):
		return self.gyro_read_axis(Itg3200.REG_GYRO_XOUT_H, Itg3200.REG_GYRO_XOUT_L, self.x_offset)

	def read_y(self):
		return self.gyro_read_axis(Itg3200.REG_GYRO_YOUT_H, Itg3200.REG_GYRO_YOUT_L, self.y_offset)

	def read_z(self):
		return self.gyro_read_axis(Itg3200.REG_GYRO_ZOUT_H, Itg3200.REG_GYRO_ZOUT_L, self.z_offset)




	def read_sample(self):
		# Ensure we're the current I2C device
		with I2cDevice.lock:
			self.set_i2c_device()

			x = self.read_x()
			y = self.read_y()
			z = self.read_z()
			
			return x, y, z


	@staticmethod
	def setup_parser():
		parser = argparse.ArgumentParser()

		parser.add_argument('-i', '--i2c-device', action="store", type=str, default='/dev/i2c-1', help="I2C Device to use")
		parser.add_argument('-c', '--calibrate', action="store_true", default=False, help="Enables manual offset calibration")
		parser.add_argument('-m', '--mode', action="store", type=str, default='g', choices=['raw', 'deg'], help="Mode of operation")
		parser.add_argument('-n', '--num-samples', action="store", type=int, default=0, help="Specifies a limit on number of samples - Default is infinity")
		parser.add_argument('-d', '--delay', action="store", type=float, default=None, help="Specifies an additional delay in seconds")
		parser.add_argument('-v', '--verbose', action="store_true", default=False, help="Enable verbose logging")
		
		return parser

	@staticmethod
	def main(argv):
		parser = Itg3200.setup_parser()
		args = parser.parse_args(argv[1:])

		# TODO: Handle kill/term/int gracefully
		
		i2c_fd = open(args.i2c_device, 'rw')

		itg3200= Itg3200(i2c_fd)

		if args.calibrate:
			itg3200.calibrate()
	
		idx = -1
		while idx < args.num_samples:
				values = itg3200.read_sample()
				if args.mode == 'g':
					values = [itg3200.adc_to_angle(val) for val in values]
					print 'Values are  :%ddeg  %ddeg  %ddeg' % (values[0], values[1], values[2])
				else:
					print 'Values are  :%d  %d  %d' % (values[0], values[1], values[2])
				if args.num_samples > 0:
					idx += 1

				if args.delay:
					time.sleep(delay)

		
		itg3200.stop()

if __name__ == "__main__":
	Itg3200.main(sys.argv)
