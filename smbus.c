#include <stdio.h>
#include "i2c-dev.h"
#include <sys/types.h>
#include <inttypes.h>

int smbus_read_byte_data(int fd, int reg) {
	return i2c_smbus_read_byte_data(fd, reg);
}

int smbus_write_byte_data(int fd, int reg, int value) {
	return i2c_smbus_write_byte_data(fd, reg, value);
}

int smbus_read_byte(int fd) {
	return i2c_smbus_read_byte(fd);
}

int smbus_write_byte(int fd, int value) {
	return i2c_smbus_write_byte(fd, value);
}

