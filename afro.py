import sys,os,re,argparse
import threading
import time
import i2c
from i2c import I2cDevice

from esc import ESC

class AfroESC(ESC):
	MOTOR_BASE = 0x29
	MAX_SPEED = 240
	DEFAULT_DELAY = 10
	MAX_DELAY = 100

	def __init__(self, i2c_fd, delay=DEFAULT_DELAY, addr=MOTOR_BASE):
		assert delay <= AfroESC.MAX_DELAY, 'Invalid delay'
		super(AfroESC, self).__init__(delay, i2c_fd, addr)

	def init(self):
		self.set_i2c_device()
		for i in range(0, 0xFFF):
			self.i2c_write(0x0)
			time.sleep(10 / 1e6)
		self.speed = 0x1

	def run(self):
		self.init()
		prev_speed = 0
		self.set_i2c_device()
		while 1:
			if prev_speed != self.speed:
				print 'Setting speed to %d' % (self.speed)
				prev_speed = self.speed
			self.i2c_write(self.speed)
			time.sleep(100 / 1e6)

	def slow_stop(self):
		while self.speed != 0:
			self.speed = 0 if self.speed < 5 else self.speed - 5
			time.sleep(100 / 1e3)

def motor_id_to_addr(motor_id):
	addr = int(str(motor_id), 0)
	if addr < 4:
		# Convert to address
		return AfroESC.MOTOR_BASE + addr
	else:
		return addr

def setup_parser():
	parser = argparse.ArgumentParser()

	parser.add_argument('-m', '--motor-id', action="store", type=str, default=AfroESC.MOTOR_BASE, help="Motor ID to use")
	parser.add_argument('-i', '--i2c-device', action="store", type=str, default='/dev/i2c-1', help="I2C Device to use")
	parser.add_argument('-d', '--delay', action="store", type=float, default=10, help="Specifies an additional delay in ms")
	parser.add_argument('-v', '--verbose', action="store_true", default=False, help="Enable verbose logging")

	return parser

def command_parser():
	command_parser = argparse.ArgumentParser()
	subparser = command_parser.add_subparsers(dest='command', title='command')

	start_cmd = subparser.add_parser('start')
	start_cmd.add_argument('motor_id', type=str, metavar='MOTOR_ID', help='start MOTOR_ID')
	start_cmd.set_defaults(func=cmd_start)

	speed_cmd = subparser.add_parser('speed')
	speed_cmd.add_argument('motor_id', type=str, metavar='MOTOR_ID', help='Set speed for MOTOR_ID')
	speed_cmd.add_argument('speed', type=int, choices=range(1, AfroESC.MAX_SPEED), help='Set speed for MOTOR_ID')
	speed_cmd.set_defaults(func=cmd_speed)

	delay_cmd = subparser.add_parser('delay')
	delay_cmd.add_argument('motor_id', type=str, metavar='MOTOR_ID', help='Set delay for MOTOR_ID')
	delay_cmd.add_argument('delay', type=int, choices=range(100, AfroESC.MAX_DELAY), help='Set delay for MOTOR_ID')
	delay_cmd.set_defaults(func=cmd_delay)

	exit_cmd = subparser.add_parser('exit')
	exit_cmd.set_defaults(func=cmd_exit)

	return command_parser


def ESC_worker(args):
	addr = motor_id_to_addr(args.motor_id)
	print 'Starting ESC with ID: 0x%X' % (addr)
	i2c_fd = open(args.i2c_device, 'rw')
	esc = AfroESC(i2c_fd, delay=args.delay, addr=addr)
	global esc_list
	esc_list = []
	esc_list.append(esc)
	esc.run()

def cmd_start(args):
	addr = int(str(args.motor_id), 16)
	try:
		idx = addr - AfroESC.MOTOR_BASE
		assert idx < 4
		esc = esc_list[idx]
		assert esc
	except Exception, e:
		print repr(e)
		return -1
	esc.init()
	return 0

def cmd_exit(args):
	for esc in esc_list:
		esc.slow_stop()
	sys.exit(0)

def cmd_speed(args):
	addr = motor_id_to_addr(args.motor_id)
	speed = args.speed
	try:
		idx = addr - AfroESC.MOTOR_BASE
		assert idx < 4
		esc = None
		for e in esc_list:
			if e.addr == addr:
				esc = e
		assert esc
		assert speed > 1 and speed < esc.MAX_SPEED
	except Exception, e:
		print repr(e)
		return -1

	esc.speed = speed

def cmd_delay(args):
	addr = motor_id_to_addr(args.motor_id)
	delay = args.delay
	try:
		idx = addr - AfroESC.MOTOR_BASE
		assert idx < 4
		for e in esc_list:
			if e.addr == addr:
				esc = e
		assert esc
		assert delay > 1 and delay <= esc.MAX_DELAY
	except Exception, e:
		print repr(e)
		return -1

	esc.delay = delay


def shell_thread():
	cmd_parser = command_parser()
	while 1:
		try:
			string = raw_input('$ ')
			string = string.strip()
			if not string:
				continue
			tokens = string.split(' ')
			args = cmd_parser.parse_args(tokens)
			if args:
				args.func(args)
		except KeyboardInterrupt:
			return

def main(argv):
	parser = setup_parser()
	args = parser.parse_args(argv[1:])

	worker = threading.Thread(target=ESC_worker, args=(args,))
	worker.daemon = True
	worker.start()

	shell_worker = threading.Thread(target=shell_thread)
	shell_worker.daemon = True
	shell_worker.start()

	while True:
		time.sleep(0.3)


if __name__ == '__main__':
	main(sys.argv)
