import os, sys, argparse
import time
import numpy

from i2c import I2cDevice
from imu import Accelerometer

class Adxl345(Accelerometer):
	ADXL345_ADDRESS                  = 0x53        #I2C address of ADXL345

	REG_DEVID                        = 0x00        #Device ID Register
	REG_THRESH_TAP                   = 0x1D        #Tap Threshold
	REG_OFFSET_X                     = 0x1E        #X-axis offset
	REG_OFFSET_Y                     = 0x1F        #Y-axis offset
	REG_OFFSET_Z                     = 0x20        #Z-axis offset
	REG_TAP_DURATION                 = 0x21        #Tap Duration
	REG_TAP_LATENCY                  = 0x22        #Tap latency
	REG_TAP_WINDOW                   = 0x23        #Tap window
	REG_THRESH_ACTIVITY              = 0x24        #Activity Threshold
	REG_THRESH_INACTIVITY            = 0x25        #Inactivity Threshold
	REG_TIME_INACTIVITY              = 0x26        #Inactivity Time
	REG_ACTIVITY_INACTIVITY_CTL      = 0x27        #Axis enable control for activity and inactivity detection
	REG_THRESH_FREEFALL              = 0x28        #free-fall threshold
	REG_TIME_FREEFALL                = 0x29        #Free-Fall Time
	REG_TAP_AXES                     = 0x2A        #Axis control for tap/double tap
	REG_ACTIVITY_TAP_STATUS          = 0x2B        #Source of tap/double tap
	REG_BW_RATE                      = 0x2C        #Data rate and power mode control
	REG_POWER_CTL                    = 0x2D        #Power Control Register
	REG_INT_ENABLE                   = 0x2E        #Interrupt Enable Control
	REG_INT_MAP                      = 0x2F        #Interrupt Mapping Control
	REG_INT_SOURCE                   = 0x30        #Source of interrupts
	REG_DATA_FORMAT                  = 0x31        #Data format control
	REG_DATA_X_L                     = 0x32        #X-Axis Data 0
	REG_DATA_X_H                     = 0x33        #X-Axis Data 1
	REG_DATA_Y_L                     = 0x34        #Y-Axis Data 0
	REG_DATA_Y_H                     = 0x35        #Y-Axis Data 1
	REG_DATA_Z_L                     = 0x36        #Z-Axis Data 0
	REG_DATA_Z_H                     = 0x37        #Z-Axis Data 1
	REG_FIFO_CTL                     = 0x38        #FIFO control
	REG_FIFO_STATUS                  = 0x39        #FIFO status

	ADXL345_RESOLUTION               = 0.004       #mg/lsb

	# Output Data Rates(Hz)
	BITS_RATE_3200HZ    = 0x0F
	BITS_RATE_1600HZ    = 0x0E
	BITS_RATE_800HZ     = 0x0D
	BITS_RATE_400HZ     = 0x0C
	BITS_RATE_200HZ     = 0x0B
	BITS_RATE_100HZ     = 0x0A

	# Power management bits
	BITS_PWR_LINK       = 1<<5
	BITS_PWR_AUTO_SLEEP = 1<<4
	BITS_PWR_MEASURE    = 1<<3
	BITS_PWR_SLEEP      = 1<<2
	BITS_PWR_WAKEUP_8Hz = 0x00
	BITS_PWR_WAKEUP_4Hz = 0x01
	BITS_PWR_WAKEUP_2Hz = 0x10
	BITS_PWR_WAKEUP_1Hz = 0x11          

	# Data format bits
	BITS_DATA_FULL_RES  = 1<<3
	BITS_DATA_RANGE_2G  = 0x00
	BITS_DATA_RANGE_4G  = 0x01
	BITS_DATA_RANGE_8G  = 0x10
	BITS_DATA_RANGE_16G = 0x11

	#Scaling

	# The base scale is the value for scaling under +/- 2g range. 
	# This is also the default value under BITS_DATA_FULL_RES.
	# When BITS_DATA_FULL_RES is disabled, the scale changes based on the range specified.
	# +/- 4g  is BASE_SCALE * 2
	# +/- 8g  is BASE_SCALE * 4
	# +/- 16g is BASE_SCALE * 8
	BASE_SCALE = 0.0039



	# FIFO_CTL bits
	FIFO_BYPASS  = 0x00
	FIFO_STORE   = 0x01
	FIFO_STREAM  = 0x10
	FIFO_TRIGGER = 0x11


	def __init__(self, i2c_fd, addr=ADXL345_ADDRESS, accel_range=BITS_DATA_RANGE_4G):
		super(Adxl345, self).__init__(i2c_fd, addr, accel_range)
	
	def init(self):
		self.set_i2c_device()
		self.i2c_write_register(Adxl345.REG_DATA_FORMAT, Adxl345.BITS_DATA_FULL_RES | self.accel_range)
		self.i2c_write_register(Adxl345.REG_FIFO_CTL, Adxl345.FIFO_STREAM)
		self.i2c_write_register(Adxl345.REG_BW_RATE, Adxl345.BITS_RATE_400HZ)
		self.i2c_write_register(Adxl345.REG_POWER_CTL, Adxl345.BITS_PWR_MEASURE)

	def calibrate(self, loop=100, sleep_period=I2cDevice.sleep_period):
		self.i2c_write_register(Adxl345.REG_POWER_CTL, 0x0)
		self.i2c_write_register(Adxl345.REG_OFFSET_X, 0x0)
		self.i2c_write_register(Adxl345.REG_OFFSET_Y, 0x0)
		self.i2c_write_register(Adxl345.REG_OFFSET_Z, 0x0)
		
		self.init()

		x_accel, y_accel, z_accel = 0, 0, 0
		x_tmp, y_tmp, z_tmp = 0, 0, 0
		for i in range(0, loop):
			x = self.read_x()
			y = self.read_y()
			z = self.read_z()
#			print '%d  %d  %d' % (x, y, z)
			x_tmp += x
			y_tmp += y
			z_tmp += z
			time.sleep(sleep_period)

		# TODO: Verify that the following logic is correct for all 'accel_range' values
		x_accel = numpy.uint16((x_accel - (x_tmp / loop)) / 4)
		y_accel = numpy.uint16((y_accel - (y_tmp / loop)) / 4)
		z_accel = numpy.uint16((z_accel - (z_tmp / loop)) / 4)

		# Fix the offsets
		self.i2c_write_register(Adxl345.REG_POWER_CTL, 0x00)
		self.i2c_write_register(Adxl345.REG_OFFSET_X, x_accel)
		self.i2c_write_register(Adxl345.REG_OFFSET_Y, y_accel)
		self.i2c_write_register(Adxl345.REG_OFFSET_Z, z_accel)

		# Restore measuring mode
		self.i2c_write_register(Adxl345.REG_POWER_CTL, Adxl345.BITS_PWR_MEASURE)

	def stop(self):
		self.i2c_write_register(Adxl345.REG_POWER_CTL, 0x0)

	# Common routine to read a particular axis
	def read_axis(self, axis_h, axis_l):
		h, l = 0, 0

		h = self.i2c_read_register(axis_h)
		l = self.i2c_read_register(axis_l)
		value = (h << 8) | l
		if value & (1 << 16 - 1):
			value = value - (1<<16)

		return float(value)

	def read_x(self):
		return self.read_axis(Adxl345.REG_DATA_X_H, Adxl345.REG_DATA_X_L)

	def read_y(self):
		return self.read_axis(Adxl345.REG_DATA_Y_H, Adxl345.REG_DATA_Y_L)

	def read_z(self):
		return self.read_axis(Adxl345.REG_DATA_Z_H, Adxl345.REG_DATA_Z_L)

	def adc_to_g(self, value):
		return value * (Adxl345.BASE_SCALE * self.accel_range)
	
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
		parser.add_argument('-m', '--mode', action="store", type=str, default='g', choices=['raw', 'g', 'all'], help="Mode of operation")
		parser.add_argument('-n', '--num-samples', action="store", type=int, default=0, help="Specifies a limit on number of samples - Default is infinity")
		parser.add_argument('-d', '--delay', action="store", type=float, default=None, help="Specifies an additional delay in seconds")
		parser.add_argument('-v', '--verbose', action="store_true", default=False, help="Enable verbose logging")
		
		return parser

	@staticmethod
	def main(argv):
		parser = Adxl345.setup_parser()
		args = parser.parse_args(argv[1:])

		# TODO: Handle kill/term/int gracefully
		
		i2c_fd = open(args.i2c_device, 'rw')

		adxl345 = Adxl345(i2c_fd)
		adxl345.init()

		if args.calibrate:
			adxl345.calibrate()
	
		idx = -1
		while idx < args.num_samples:
				values = adxl345.read_sample()
				if args.mode == 'g':
					values = [adxl345.adc_to_g(val) for val in values]
					print 'Values are  :%7.3fg  %7.3fg  %7.3fg' % (values[0], values[1], values[2])
				elif args.mode == 'all':
					g_values = [adxl345.adc_to_g(val) for val in values]
					print 'Values are  :%d(%7.3fg)  %d(%7.3fg)  %d(%7.3fg)' % (values[0], g_values[0], values[1], g_values[1], values[2], g_values[2])
				else:
					print 'Values are  :%d  %d  %d' % (values[0], values[1], values[2])
				if args.num_samples > 0:
					idx += 1

				time.sleep(adxl345.sleep_period)
				if args.delay:
					time.sleep(args.delay)

		adxl345.stop()

		
if __name__ == "__main__":
	Adxl345.main(sys.argv)
