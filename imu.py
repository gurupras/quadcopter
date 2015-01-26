import i2c

class Imu(I2cDevice):
	event_listener_list = []
	i2c_fd

	def __init__(self, i2c_fd):
		super(Imu, self).__init__(i2c_fd)

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
	accelerometer
	gyroscope
	magnetometer
	def __init__(self, accelerometer, gyroscope, magnetometer):
		self.accelerometer = accelerometer
		self.gyroscope     = gyroscope
		self.magnetometer  = magnetometer
	
class Accelerometer(Imu):
	accel_range

	def __init__(self, i2c_fd, accel_range):
		super(Accelerometer, self).__init__(i2c_fd)
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


class Gyroscope(Imu):

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
	
