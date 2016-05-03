# CC=gcc
# Use the following (instead of gcc) for arm compiler
CC?=gcc
# Use the following (instead of gcc) for arm compiler
ifeq ($(CROSS_COMPILE), 1)
	CC =/opt/bbToolChain/usr/local/share/codesourcery/bin/arm-none-linux-gnueabi-gcc
endif
CFLAGS= -O3 -Wall -DPROFILE -mfloat-abi=softfp -mfpu=neon
LIBS= -lm
LDFLAGS=
EXEC = matMult
CSRCS = matrixmult.c Timer.c
OBJS = $(CSRCS:.c=.o)

# Remote directories
BEAGLE_DIR=/home/root/esLAB/assignment-1
ESL_DIR=/data/home/in4342-03/embedded-systems-lab/assignment-1/neon
REMOTE_BB?=root@192.168.0.202
REMOTE_ESL=in4342-03@ce-eslab.ewi.tudelft.nl

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(CFLAGS)  -o $@ $? $(LIBS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(EXEC)
	./$(EXEC)

clean:
	@-rm -rf $(EXEC) $(OBJS) *~

############# REMOTE STUFF ################
# Fetch the files from remote
fetch:
	scp $(REMOTE_ESL):$(ESL_DIR)/{$(FETCH_FILES)} ./

# Build it remote
remote:
	scp ./* $(REMOTE_ESL):$(ESL_DIR)/
	ssh -C $(REMOTE_ESL) "cd $(ESL_DIR); make clean && make CROSS_COMPILE=1"

# Send it trough SCP
send: $(EXEC)
	echo "Sending $(EXEC) to beagleboard ... "
	scp -P$(PORT) $(EXEC) $(REMOTE_BB):$(BEAGLE_DIR)/.

.PHONY: all canny run clean send
