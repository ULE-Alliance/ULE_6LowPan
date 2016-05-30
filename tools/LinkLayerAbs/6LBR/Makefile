#################################################
# Makefile for RTX uleapp on uClinux
#################################################

INCLUDEDIRS := \
	../include \

CFLAGS += $(addprefix -I,$(INCLUDEDIRS))

SRCS += \
    lla_6LBR.c	

OBJS = $(SRCS:.c=.o)



# link
6LBR: $(OBJS)
	gcc $(OBJS) -o 6LBR

# pull in dependency info for *existing* .o files
-include $(OBJS:.o=.d)

# compile and generate dependency info
%.o: %.c
	gcc -c $(CFLAGS) $*.c -o $*.o
	gcc -MM $(CFLAGS) $*.c > $*.d

# remove compilation products
clean:
	rm -f 6LBR *.o *.d