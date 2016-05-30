#################################################
# Makefile for RTX uleapp on uClinux
#################################################

INCLUDEDIRS := \
	../include \

CFLAGS += $(addprefix -I,$(INCLUDEDIRS))

SRCS += \
    lla_6LN.c	

OBJS = $(SRCS:.c=.o)



# link
6LN: $(OBJS)
	gcc $(OBJS) -o 6LN

# pull in dependency info for *existing* .o files
-include $(OBJS:.o=.d)

# compile and generate dependency info
%.o: %.c
	gcc -c $(CFLAGS) $*.c -o $*.o
	gcc -MM $(CFLAGS) $*.c > $*.d

# remove compilation products
clean:
	rm -f 6LN *.o *.d