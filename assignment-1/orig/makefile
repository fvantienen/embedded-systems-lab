PIN_DIR=$(PIN_ROOT)
MCPROF_DIR=/data/home/in4342-03/mcprof

CC=gcc
# Use the following (instead of gcc) for arm compiler
# CC=/data/usr/local/share/codesourcery/bin/arm-none-linux-gnueabi-gcc
CFLAGS= -O3 -Wall
LIBS= -lm
LDFLAGS=
EXEC = matMult
CMD=./$(EXEC)
CSRCS = matrixmult.c Timer.c
OBJS = $(CSRCS:.c=.o)

# Fetched files
FETCH_FILES = $(EXEC),callgraph.pdf,communication.pdf

# Remote directories
BEAGLE_DIR=/home/root/esLAB/matMULT
ESL_DIR=/data/home/in4342-03/embedded-systems-lab/assignment-1/orig

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(CFLAGS)  -o $@ $? $(LIBS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(EXEC)
	$(CMD)

clean:
	@-rm -rf $(EXEC) $(OBJS) *~ gprof.txt

#MCProf part
MCPROF_OPT:=-RecordStack 0  -TrackObjects 1 -Engine 2
compile4mcprof: CFLAGS= -O1 -g -fno-inline
compile4mcprof: clean $(EXEC)

mcprof: compile4mcprof
	$(PIN_DIR)/pin -t $(MCPROF_DIR)/obj-intel64/mcprof.so $(MCPROF_OPT) -- $(CMD)
	$(MCPROF_DIR)/scripts/dot2pdf.sh communication.dot
	$(MCPROF_DIR)/scripts/plotScript.sh
	$(MCPROF_DIR)/scripts/callgraph2pdf.sh

#GProf part
compile4gprof: CFLAGS= -O1 -g -pg -fno-inline
compile4gprof: LDFLAGS= -pg
compile4gprof: clean $(EXEC)
gprof: compile4gprof
	$(CMD)
	gprof -b $(EXEC) > gprof.txt

############# REMOTE STUFF ################
# Fetch the files from remote
fetch:
	scp in4342-03@ce-eslab.ewi.tudelft.nl:$(ESL_DIR)/{$(FETCH_FILES)} ./

# Send it trough SCP
send: $(EXEC)
	echo "Sending $(EXEC) to beagleboard ... "
	scp $(EXEC) root@192.168.0.202:$(BEAGLE_DIR)/.

.PHONY: all canny run clean send
