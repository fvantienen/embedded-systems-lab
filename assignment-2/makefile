# set the path to pin if not set in .bachrc
PIN_DIR=/opt/pin-2.14-71313
MCPROF_DIR=/data/home/in4342-03/mcprof

CC?=gcc
# Use the following (instead of gcc) for arm compiler
ifeq ($(CROSS_COMPILE), 1)
	CC =/opt/bbToolChain/usr/local/share/codesourcery/bin/arm-none-linux-gnueabi-gcc
endif
INC = -I.
CFLAGS =-O2 -Wall
LFLAGS = -L.
LIBS = -lm
LDFLAGS=

CSRCS 	= canny_edge.c hysteresis.c pgm_io.c Timer.c
OBJS	=$(CSRCS:%.c=%.o)

# Use one of the following pictures
PIC	?=klomp.pgm
# PIC	=square.pgm
# PIC	=tiger.pgm

EXEC    = canny
CMD	= ./$(EXEC) ./pics/$(PIC)

# Fetched files
FETCH_FILES = $(EXEC),callgraph.pdf,communication.pdf

# Remote directories
BEAGLE_DIR=/home/root/esLAB/assignment-2
ESL_DIR=/data/home/in4342-03/embedded-systems-lab/assignment-2
REMOTE_BB?=root@192.168.0.202
REMOTE_ESL=in4342-03@ce-eslab.ewi.tudelft.nl

all: clean $(EXEC) run

$(EXEC) : $(OBJS)
	$(CC) $(OBJS) -o $(EXEC) $(LFLAGS) $(LDFLAGS) $(LIBS)

%.o : %.c
	$(CC) $(CFLAGS) $(INCS) -c $< -o $@ 

run: $(EXEC) $(PIC)
	$(CMD)

debug: CFLAGS= -g -Wall
debug: clean $(EXEC) $(PIC)
	gdb --args $(CMD)

gprof: CFLAGS=-O2 -g -pg -Wall
gprof: LDFLAGS=-pg
gprof: clean $(EXEC)
	$(CMD)
	gprof -b $(EXEC) > gprof.out

MCPROF_OPT:=-RecordStack 0  -TrackObjects 1 -Engine 2 -TrackStartStop 1 -StaticSymbols 0 -TrackZones 0 -Threshold 100
mcprof: CFLAGS= -O1 -g -fno-inline -fno-omit-frame-pointer -Wall
mcprof: clean $(EXEC)
	$(PIN_DIR)/pin -t $(MCPROF_DIR)/obj-intel64/mcprof.so $(MCPROF_OPT) -- $(CMD)
	$(MCPROF_DIR)/scripts/callgraph2pdf.sh
	$(MCPROF_DIR)/scripts/dot2pdf.sh communication.dot
	$(MCPROF_DIR)/scripts/plotScript.sh

clean:
	rm -f $(EXEC) $(OBJS) *~ gmon.out pin.log *.out *.dot *.json *.eps

############# REMOTE STUFF ################
# Fetch the files from remote
fetch:
	scp -r $(REMOTE_ESL):$(ESL_DIR)/{$(FETCH_FILES)} ./

# Build it remote
remote:
	scp -r ./* $(REMOTE_ESL):$(ESL_DIR)/
	ssh -C $(REMOTE_ESL) "cd $(ESL_DIR); make clean && make CROSS_COMPILE=1"

# Build it remote
remote-prof:
	scp -r ./* $(REMOTE_ESL):$(ESL_DIR)/
	ssh -C $(REMOTE_ESL) "cd $(ESL_DIR); make clean && make mcprof"
	ssh -C $(REMOTE_ESL) "cd $(ESL_DIR); make clean && make gprof"

# Send it trough SCP
send: $(EXEC)
	echo "Sending $(EXEC) to beagleboard ... "
	scp -P$(PORT) $(EXEC) $(REMOTE_BB):$(BEAGLE_DIR)/.

.PHONY: clean canny run all debug gprof mcprof fetch remote remote-prof send
