CC=gcc
# Use the following (instead of gcc) for arm compiler
# CC=/data/usr/local/share/codesourcery/bin/arm-none-linux-gnueabi-gcc
CFLAGS= -O3 -Wall
LIBS= -lm
LDFLAGS=
EXEC = matMult
CSRCS = matrixmult.c Timer.c
OBJS = $(CSRCS:.c=.o)

# make sure following dir exists on beagleboard
BEAGLE_DIR=/home/root/esLAB/matMULT

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(CFLAGS)  -o $@ $? $(LIBS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(EXEC)
	./$(EXEC)

clean:
	@-rm -rf $(EXEC) $(OBJS) *~

send: $(EXEC)
	echo "Sending $(EXEC) to beagleboard ... "
	scp $(EXEC) root@192.168.0.202:$(BEAGLE_DIR)/.

.PHONY: all canny run clean send
