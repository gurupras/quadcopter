LIB_NAME=smbus
MAKEFLAGS = --no-builtin-rules
# We don't use any suffix rules
.SUFFIXES :

CFLAGS = -fpic
CC_OPTS = $(CFLAGS) $($*_CC_OPTS) -Wall -Werror
CC =gcc
LD =ld
AR =ar

all :
	swig -python smbus.i
	CFLAGS="-O3 -Werror" python setup.py build_ext --inplace

%.o : %.c
	$(addprefix $(CROSS), $(CC)) $(CC_OPTS) -c $< -o $@ $(LIBS)

clean :
	@rm -rf *.o **/*.o $(LIB_NAME).so $(LIB_NAME).a $(LIB_NAME) $(LIB_NAME).py $(LIB_NAME)_wrap.c build *.pyc

.PHONY : clean all

