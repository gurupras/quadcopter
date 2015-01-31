from i2c import I2cDevice

from abc import abstractmethod

class IMU(I2cDevice):
	event_listener_list = []

	def __init__(self, i2c_fd, addr):
		super(IMU, self).__init__(i2c_fd, addr)

	def register_event_listener(self, func):
		self.event_listener_list.append(func)
	
	def unregister_event_listener(self, func):
		self.event_listener_list.remove(func)

	@abstractmethod
	def on_event(self, event_obj):
		raise Exception('Unimplemented')

	@abstractmethod
	def sensor_loop(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def calibrate(self, loop, sleep):
		raise Exception('Unimplemented')

class IMU_9DOF:
	def __init__(self, accelerometer, gyroscope, magnetometer):
		self.accelerometer = accelerometer
		self.gyroscope     = gyroscope
		self.magnetometer  = magnetometer
	
class Accelerometer(IMU):
	def __init__(self, i2c_fd, addr, accel_range):
		super(Accelerometer, self).__init__(i2c_fd, addr)
		self.accel_range = accel_range
	
	@abstractmethod
	def read_x(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def read_y(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def read_z(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def adc_to_g(self):
		raise Exception('Unimplemented')


class Gyroscope(IMU):

	def __init__(self, i2c_fd):
		super(Gyroscope, self).__init__(i2c_fd)

	@abstractmethod
	def read_x(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def read_y(self):
		raise Exception('Unimplemented')

	@abstractmethod
	def read_z(self):
		raise Exception('Unimplemented')

	def adc_to_angle(self):
		raise Exception('Unimplemented')
	
