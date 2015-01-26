%module smbus

%{
#define SWIG_FILE_WITH_INIT
#include "smbus.h"
%}

%include "smbus.h"
int32_t smbus_read_byte(int fd, int reg);
int32_t smbus_write_byte(int fd, int reg, int value);

