#   ----------------------------------------------------------------------------
#  @file   Makefile
#
#  @path   
#
#  @desc   Makefile for helloDSPgpp (Debug and Release versions)
#
#  @ver    1.10
#   ----------------------------------------------------------------------------
#  Copyright (c) Texas Instruments Incorporated 2002-2009
#
#  Use of this software is controlled by the terms and conditions found in the
#  license agreement under which this software has been supplied or provided.
#   ----------------------------------------------------------------------------
SHELL = /bin/sh

#   ----------------------------------------------------------------------------
#   Base for toolchain
#   This may change depending if you are using Montavista or Codesourcery
#   ----------------------------------------------------------------------------
#BASE_TOOLCHAIN := /opt/mv_pro_5.0/montavista/pro/devkit/arm/v5t_le
#BASE_TOOLCHAIN := /opt/arm/arm-2007q3       # Codesourcery tools
#BASE_TOOLCHAIN := /usr/local/share/codesourcery/
#BASE_TOOLCHAIN := /data/usr/local/share/codesourcery/
BASE_TOOLCHAIN := /opt/bbToolChain/usr/local/share/codesourcery/
#   ----------------------------------------------------------------------------
#   Name of the Linux compiler
#   This may change depending if you are using Montavista or Codesourcery
#   ----------------------------------------------------------------------------
#CC := arm_v5t_le-gcc
CC := arm-none-linux-gnueabi-gcc
#CC := arm-none-linux-gnueabi-gcc

#   ----------------------------------------------------------------------------
#   Base for DSPLink
#   This may have to be changed depending on
#   your installation and DSPLink version
#   ----------------------------------------------------------------------------
#DSPLINK := /home/user/OMAPL137_arm_1_00_00_10/dsplink-1_61_03-prebuilt/packages/dsplink
#DSPLINK := /data/usr/local/share/bbframework/platform/beagle-linux/tools/dsplink_linux_1_65_00_03
DSPLINK := /opt/bbToolChain/usr/local/share/bbframework/platform/beagle-linux/tools/dsplink_linux_1_65_00_03
# DSPLINK := /data/ESLab/DSPLink/dsplink_linux_1_63/dsplink

#   ----------------------------------------------------------------------------
#   General options, sources and libraries
#   ----------------------------------------------------------------------------
SRCS := helloDSP.c system_os.c main.c
OBJS :=
DEBUG :=
LDFLAGS := -lpthread
CFLAGS := -DPROFILE -DVERIFY_DATA
LIBS :=
BIN := helloDSPgpp

#   ----------------------------------------------------------------------------
#   Compiler and Linker flags for Debug
#   ----------------------------------------------------------------------------
OBJDIR_D := Debug
BINDIR_D := $(OBJDIR_D)
#LIBS_D := $(DSPLINK)/gpp/export/BIN/Linux/OMAPL1XX/DEBUG/dsplink.lib $(LIBS)
LIBS_D := $(DSPLINK)/gpp/BUILD/EXPORT/DEBUG/dsplink.lib $(LIBS)
# If the DSP/Link was rebuilt by the user, replace the line above 
# with the one below to use the updated libraries
#LIBS_D := $(DSPLINK)/gpp/BUILD/EXPORT/DEBUG/dsplink.lib $(LIBS)
OBJS_D := $(SRCS:%.c=$(OBJDIR_D)/%.o)
ALL_DEBUG := -g -DDDSP_DEBUG $(DEBUG) -D__DEBUG

#   ----------------------------------------------------------------------------
#   Compiler and Linker flags for Release
#   ----------------------------------------------------------------------------
OBJDIR_R := Release
BINDIR_R := $(OBJDIR_R)
#LIBS_R := $(DSPLINK)/gpp/export/BIN/Linux/OMAPL1XX/RELEASE/dsplink.lib $(LIBS)
# If the DSP/Link was rebuilt by the user, replace the line above 
# with the one below to use the updated libraries
LIBS_R := $(DSPLINK)/gpp/BUILD/EXPORT/RELEASE/dsplink.lib $(LIBS)
OBJS_R := $(SRCS:%.c=$(OBJDIR_R)/%.o)

#   ----------------------------------------------------------------------------
#   Compiler include directories 
#   ----------------------------------------------------------------------------
INCLUDES := -I$(DSPLINK)/gpp/inc                   \
            -I$(DSPLINK)/gpp/inc/usr               \
            -I$(DSPLINK)/gpp/inc/sys/Linux         \
            -I$(DSPLINK)/gpp/inc/sys/Linux/2.6.18  \
            -I$(BASE_TOOLCHAIN)/include \
            -I./

#   ----------------------------------------------------------------------------
#   All compiler options to be passed to the command line
#   ----------------------------------------------------------------------------
ALL_CFLAGS := $(INCLUDES)                   \
              -mlittle-endian               \
              -march=armv5t                 \
              -mtune=arm9tdmi               \
              -msoft-float                  \
              -Uarm                         \
              -Wdeclaration-after-statement \
              -marm -Wall                   \
              -Wstrict-prototypes           \
              -Wno-trigraphs                \
              -fno-strict-aliasing          \
              -fno-common                   \
              -fno-omit-frame-pointer       \
              -mapcs                        \
              -c                            \
              -mabi=aapcs-linux             \
              -O                            \
              $(CFLAGS)

#   ----------------------------------------------------------------------------
#   Compiler symbol definitions 
#   ----------------------------------------------------------------------------

DEFS :=        -DOS_LINUX            \
               -DMAX_DSPS=1          \
               -DMAX_PROCESSORS=2    \
               -DID_GPP=1            \
               -DOMAPL1XX            \
               -DPROC_COMPONENT      \
               -DPOOL_COMPONENT      \
               -DNOTIFY_COMPONENT    \
               -DMPCS_COMPONENT      \
               -DRINGIO_COMPONENT    \
               -DMPLIST_COMPONENT    \
               -DMSGQ_COMPONENT      \
               -DMSGQ_ZCPY_LINK      \
               -DCHNL_COMPONENT      \
               -DCHNL_ZCPY_LINK      \
               -DZCPY_LINK           \
               -DKFILE_DEFAULT       \
               -DDA8XXGEM            \
               -DDA8XXGEM_PHYINTERFACE=SHMEM_INTERFACE

# Fetched files
EXEC = $(BINDIR_R)/$(BIN)

# Remote directories
BEAGLE_DIR=/home/root/esLAB/assignment-1
ESL_DIR=/data/home/in4342-03/embedded-systems-lab/assignment-1/gpp
REMOTE_BB=root@192.168.0.202
REMOTE_ESL=in4342-03@ce-eslab.ewi.tudelft.nl

#   ----------------------------------------------------------------------------
#   Compiler and Linker procedure
#   From this point and on changes are very unlikely.
#   ----------------------------------------------------------------------------
.PHONY: all
all: Debug Release

#   ----------------------------------------------------------------------------
#   Building Debug... 
#   ----------------------------------------------------------------------------
.PHONY: Debug
Debug: $(BINDIR_D)/$(BIN)

$(BINDIR_D)/$(BIN): $(OBJS_D)
	@echo Compiling Debug...
	@$(BASE_TOOLCHAIN)/bin/$(CC) -o $@ $(OBJS_D) $(LIBS_D) $(LDFLAGS)

$(OBJDIR_D)/%.o : %.c
	@$(BASE_TOOLCHAIN)/bin/$(CC) $(ALL_DEBUG) $(DEFS) $(ALL_CFLAGS) -o$@ $<

#   ----------------------------------------------------------------------------
#   Building Release... 
#   ----------------------------------------------------------------------------
.PHONY: Release
Release: $(BINDIR_R)/$(BIN)

$(BINDIR_R)/$(BIN): $(OBJS_R)
	@echo Compiling Release...
	@$(BASE_TOOLCHAIN)/bin/$(CC) -o $@ $(OBJS_R) $(LIBS_R) $(LDFLAGS)

$(OBJDIR_R)/%.o : %.c
	@$(BASE_TOOLCHAIN)/bin/$(CC) $(DEFS) $(ALL_CFLAGS) -o$@ $<


############# REMOTE STUFF ################
# Fetch the files from remote
fetch:
	scp $(REMOTE_ESL):$(ESL_DIR)/$(EXEC) ./Release/

# Build it remote
remote:
	scp -r ./* $(REMOTE_ESL):$(ESL_DIR)/
	ssh -C $(REMOTE_ESL) "cd $(ESL_DIR); make clean && make"

# Send it trough SCP
send:
	echo "Sending $(EXEC) to beagleboard ... "
	scp $(EXEC) $(REMOTE_BB):$(BEAGLE_DIR)/.

.PHONY: clean
clean:
	@rm -f $(OBJDIR_D)/*
	@rm -f $(OBJDIR_R)/*

