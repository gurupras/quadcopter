LIB_NAME=smbus
MAKEFLAGS = --no-builtin-rules
# We don't use any suffix rules
.SUFFIXES :

CFLAGS = -fpic
CC_OPTS = $(CFLAGS) $($*_CC_OPTS) -Wall -Werror
CC =gcc
LD =ld
AR =ar

all : smbus.o
	$(addprefix $(CROSS), $(CC)) $(CFLAGS) -shared -o lib$(LIB_NAME).so $^ $(LIBS)
	$(addprefix $(CROSS), $(AR)) rcs lib$(LIB_NAME).a $^

%.o : %.c
	$(addprefix $(CROSS), $(CC)) $(CC_OPTS) -c $< -o $@ $(LIBS)

clean :
	@rm -rf *.o **/*.o $(LIB_NAME).so $(LIB_NAME).a $(LIB_NAME)

.PHONY : clean all

