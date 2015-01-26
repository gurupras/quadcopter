#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <inttypes.h>

uint8_t smbus_read_byte(int fd, int reg) {
	return i2c_smbus_read_byte_data(fd, reg);
}

void smbus_write_byte(int fd, int reg, int value) {
	i2c_smbus_write_byte_data(fd, reg, value);
}

