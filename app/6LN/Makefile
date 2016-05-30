#################################################
# Makefile for RTX uleapp on uClinux
#################################################

.PHONY: clean deps 6LN

CONTIKI := ../../src/contiki
CONTIKI_CORE := $(CONTIKI)/core/
CONTIKI_CPU := $(CONTIKI)/cpu/
CONTIKI_PLATFORM := $(CONTIKI)/platform/
CONTIKI_APPS := $(CONTIKI)/apps/

CONTIKI_INCLUDES := \
    $(CONTIKI)/core \
    $(CONTIKI)/core/net \
    $(CONTIKI)/cpu/native \
    $(CONTIKI)/platform/ule \
    $(CONTIKI)/platform/native
    
 CONTIKI_SOURCES := \
  $(CONTIKI_CORE)net/netstack.c \
  $(CONTIKI_CORE)net/packetbuf.c \
  $(CONTIKI_CORE)net/ip/psock.c \
  $(CONTIKI_CORE)net/linkaddr.c \
  $(CONTIKI_CORE)net/nbr-table.c \
  $(CONTIKI_CORE)net/llsec/nullsec.c \
  $(CONTIKI_CORE)net/ip/resolv.c \
  $(CONTIKI_CORE)net/ip/tcpip.c \
  $(CONTIKI_CORE)net/ip/udp-socket.c \
  $(CONTIKI_CORE)net/ip/uip-debug.c \
  $(CONTIKI_CORE)net/ip/uip-packetqueue.c \
  $(CONTIKI_CORE)net/ip/uip-udp-packet.c \
  $(CONTIKI_CORE)net/ip/uiplib.c \
  $(CONTIKI_CORE)net/ip/uip-nameserver.c \
  $(CONTIKI_CORE)net/ipv6/uip6.c \
  $(CONTIKI_CORE)net/ipv6/uip-nd6.c \
  $(CONTIKI_CORE)net/ipv6/uip-ds6.c \
  $(CONTIKI_CORE)net/ipv6/uip-ds6-route.c \
  $(CONTIKI_CORE)net/ipv6/uip-mld.c \
  $(CONTIKI_CORE)net/ipv6/uip-ds6-nbr.c \
  $(CONTIKI_CORE)net/ipv6/uip-icmp6.c \
  $(CONTIKI_CORE)net/ipv6/sicslowpan.c \
  $(CONTIKI_CORE)net/mac/mac.c \
  $(CONTIKI_CORE)net/mac/nullmac.c \
  $(CONTIKI_CORE)net/mac/nullrdc-noframer.c \
  $(CONTIKI_CORE)sys/ctimer.c \
  $(CONTIKI_CORE)sys/etimer.c \
  $(CONTIKI_CORE)sys/process.c \
  $(CONTIKI_CORE)sys/rtimer.c \
  $(CONTIKI_CORE)sys/stimer.c \
  $(CONTIKI_CORE)sys/timer.c \
  $(CONTIKI_CORE)lib/list.c \
  $(CONTIKI_CORE)lib/memb.c \
  $(CONTIKI_CORE)lib/random.c \
  $(CONTIKI_CPU)native/rtimer-arch.c \
  $(CONTIKI_CPU)native/watchdog.c \
  $(CONTIKI_PLATFORM)native/clock.c \
  
  
  #$(CONTIKI_CORE)net/queuebuf.c \
  $(CONTIKI_CORE)lib/crc16.c \
  $(CONTIKI_CORE)net/ip/simple-udp.c \
  $(CONTIKI_CORE)lib/mmem.c \
  $(CONTIKI_CORE)lib/ringbuf.c \
  
  #$(CONTIKI_CORE)dev/nullradio.c \
  
  # $(CONTIKI_CORE)sys/procinit.c \
  
  #$(CONTIKI_CORE)net/rime/rime.c \
  $(CONTIKI_CORE)net/rime/announcement.c \
  $(CONTIKI_CORE)net/rime/chameleon.c \
  $(CONTIKI_CORE)net/rime/abc.c \
  $(CONTIKI_CORE)net/rime/broadcast-announcement.c \
  $(CONTIKI_CORE)net/rime/broadcast.c \
  $(CONTIKI_CORE)net/rime/chameleon-bitopt.c \
  $(CONTIKI_CORE)net/rime/channel.c \
  
  #$(CONTIKI_CORE)net/mac/framer.c \
  
  #$(CONTIKI_CORE)net/mac/nullrdc.c \
  $(CONTIKI_CORE)net/mac/framer-802154.c \
  $(CONTIKI_CORE)net/mac/frame802154.c \
  $(CONTIKI_CORE)net/mac/framer-nullmac.c \
  
  #$(CONTIKI_CORE)net/rpl/rpl-dag.c \
  $(CONTIKI_CORE)net/rpl/rpl.c \
  $(CONTIKI_CORE)net/rpl/rpl-ext-header.c \
  $(CONTIKI_CORE)net/rpl/rpl-timers.c \
  $(CONTIKI_CORE)net/rpl/rpl-mrhof.c \
  $(CONTIKI_CORE)net/rpl/rpl-icmp6.c \
  
INCLUDEDIRS := \
	../include \
	../../tools/LinkLayerAbs/6LN/ \
	../../tools/FileLogger/ \
 	../../src/6LN/ \
 	$(CONTIKI_INCLUDES)

# Add -DCONTIKI_DEBUG_PRINT=1 for debug output of contiki uIP stack
# Most of the Contiki defines are set in contiki/platform/ule/fp-conf.h file
# which is included from contiki-conf.h
CONTIKI_DEFINES := -D_FP_ -DAUTOSTART_ENABLE -DUIP_CONF_IPV6=1 -DSICSLOWPAN_COMPRESSION=2
# -DCONTIKI_DEBUG_PRINT=1 -DDEBUG=1
# -DCONTIKI_DEBUG_PRINT=1 -DDEBUG=1 -DDEBUG_PRINT=1

#CONTIKI_WITH_IPV6 = 1

#ifeq ($(WITH_6LOWPAN),1)
#CFLAGS += $(addprefix -I,$(CONTIKI_INCLUDES)) $(CONTIKI_DEFINES) -DWITH_6LOWPAN=1

CFLAGS += $(addprefix -I,$(INCLUDEDIRS))
CFLAGS += -std=c99
CFLAGS += -DNETSTACK_CONF_WITH_IPV6=1
CFLAGS += $(CONTIKI_DEFINES)
CFLAGS += -DCONTIKI=1 -DCONTIKI_TARGET_NATIVE=1
#CFLAGS += -g3
CFLAGS += -g0
CFLAGS += -Os
CFLAGS += -DNETSTACK_RADIO=ule6lo_radio_driver
CFLAGS += -DUIP_CONF_ROUTER=0
CFLAGS += -DUIP_CONF_ND6_SEND_RA=0
CFLAGS += -DLN_ROLE=1
CFLAGS += -DNETSTACK_CONF_RDC=nullrdc_noframer_driver
CFLAGS += -DSICSLOWPAN_CONF_FRAG=0
CFLAGS += -DUIP_CONF_IPV6_RPL=0
CFLAGS += -DUIP_CONF_ND6_DEF_MAXDADNS=0
CFLAGS += -DUIP_CONF_ND6_SEND_NA=0
CFLAGS += -DUIP_CONF_ND6_RTR_SOLICITATION_INTERVAL=10
#CFLAGS += -DUIP_CONF_ND6_ARO_REACHABLE_TIME=180000
CFLAGS += -DUIP_CONF_ND6_RA_RDNSS=1
CFLAGS += -DRESOLV_CONF_SUPPORTS_MDNS=0
CFLAGS += -DUIP_CONF_MLD=1
CFLAGS += -DUIP_CONF_MULTIPLE_PREFIXES=1

SRCS += \
    app_6LN.c \
    ../../tools/LinkLayerAbs/6LN/lla_6LN.c \
    ../../tools/FileLogger/FileLogger.c \
    ../../src/6LBR/not_yet_placed.c \
    ../../src/6LN/ule6lo_radio_6LN.c \
    ../../src/6LN/ule6loGI_6LN.c \
    ../../src/6LN/ule6loLLI_6LN.c \
    ../../src/6LN/ule6loOS_6LN.c \
    ../../src/6LN/ule6loTestIn_6LN.c \
    ../../tools/DebugFunctions/DebugFunctions.c \
 	$(CONTIKI_SOURCES)

OBJDIR = obj
SRC_NAMES = $(notdir $(SRCS))
OBJS := $(patsubst %.c,${OBJDIR}/%.o, $(filter %.c,$(SRC_NAMES)))
DEPS := $(OBJS:.o=.d)

# compile and generate dependency info
#obj/%.o: %.c
	#echo HIP $*
	#gcc -c $(CFLAGS) $*.c -o obj/$(notdir $*).o
	#gcc -MM $(CFLAGS) $*.c > obj/$(notdir $*).d

#%.o: %.c
#	echo HOP $*
#	gcc -c $(CFLAGS) $*.c -o obj/$(notdir $*).o
#	gcc -MM $(CFLAGS) $*.c > obj/$(notdir $*).d


# test compile
#$(OBJDIR)/%.o: %.c
#    gcc $(CFLAGS) $(CPPFLAGS) -c -o $@ $<
#    gcc $(CFLAGS) $(CPPFLAGS) -MM $@ $<	


# link
LDFLAGS=-Wl,-Map=$@.map -Wl,--cref
6LN: $(OBJS) 
	gcc $(OBJS) $(LDFLAGS) -o 6LN

deps: cleanall
	./setup_dependencies.sh $(OBJDIR) "$(SRCS)" "$(CFLAGS)"
	
# remove compilation products
clean:
	rm -f $(OBJDIR)/*.o 6LN 

cleanall: clean
	rm -f $(OBJDIR)/*.d

# pull in dependency info for *existing* .o files
-include $(DEPS)
