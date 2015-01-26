%module smbus

%include "stdint.i"

%{
#define SWIG_FILE_WITH_INIT
#include "smbus.h"
%}

%include "smbus.h"

int smbus_read_byte(int fd, int reg);
int smbus_write_byte(int fd, int reg, int value);
