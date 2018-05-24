#################################################
# Makefile
#
# Date: 9/14/2016
#
# Desription:
# contains the Makefile for the ecen 5013
# final project
#
# Author: Jeff Venicx
#
#################################################

# Compile time flags with defaults
##########################################################
# Project version
# Options are 1, 2, 3, 4
PROJECT := 1

# Host architecture
# Options are HOST, BBB or FRDM
PLATFORM := HOST

# File upload address
# Options are any valid host (with directory)
HOST_ADDR := root@192.168.7.2:

# Debug options
# Options are TRUE or FALSE
DEBUG := TRUE

# Compiler warnings
# Options are TRUE or FALSE
WARNINGS = FALSE

# Extra flags
# Options are any extra compiler flag for given host
FLAGS := NONE

# Compiler executables
##########################################################
# Construct compiler flags
CFLAGS = -std=c99 -I$(INC_DIR)
ifeq ($(DEBUG), TRUE)
 CFLAGS += -g -O0
endif
ifneq ($(FLAGS), NONE)
 CFLAGS += $(FLAGS)
endif
ifeq ($(WARNINGS), TRUE)
 CFLAGS += -Wall -Wextra
endif
LDFLAGS = -Wl,-Map=$(BUILD_DIR)/$(MAP_NAME)

ifeq ($(PROJECT), 1)
 CFLAGS += -DPROJECT_1
else ifeq ($(PROJECT), 2)
 CFLAGS += -DPROJECT_2
else ifeq ($(PROJECT), 3)
 CFLAGS += -DPROJECT_3
else ifeq ($(PROJECT), 4)
 CFLAGS += -DPROJECT_4
else
 $(error PROJECT value must be 1, 2, 3, 4)
endif

# Set compiler and code size executables
ifeq ($(PLATFORM), HOST)
 CC   := gcc
 SIZE := size
else ifeq ($(PLATFORM), BBB)
 ifeq ($(UNAME_N), beaglebone)
  CC   := gcc
  SIZE := size
 else
  CC   := arm-linux-gnueabi-gcc
  SIZE := arm-linux-gnueabi-size
 endif
else ifeq ($(PLATFORM), FRDM)
 CC   := arm-none-eabi-gcc
 SIZE := arm-none-eabi-size
else
 $(error PLATFORM value must be HOST, BBB, or FRDM. Platform is $(PLATFORM))
endif

# Compile time flag for host
CFLAGS += -D$(PLATFORM)

# Freedom Freescale flags
ifeq ($(PLATFORM), FRDM)
CFLAGS += --specs=nosys.specs
endif

# Dependency file generation
DFLAGS = -M

# Archive
AR = ar

# File transfer type
XFER = scp

# Object dump executable
DUMP := objdump -d

# Directories and search paths
##########################################################
# Search paths
VPATH = src:test

# Include directory
INC_DIR = incl

# Build directory
BUILD_DIR = build

# Binary directory
BIN_DIR = bin

# Test directory
TEST_DIR  = test

# Directory commands
MKDIR_P = mkdir -p
RM_F    = rm -f
MV_F    = mv -f
UNAME_N = uname -n

# Sources and objects
###########################################################
# Source files and project name
include sources.mk

# Associate each object file with a source
OBJS := $(SRCS:.c=.o)

# Preprocessed files
PRES := $(SRCS:.c=.i)

# Assembly files
ASSEMS := $(SRCS:.c=.s)

# Library file
LIBS_OBJS = $(LIB_SRCS:.c=.o)
LIB_NAME = libproject1.a

# Map file
MAP_NAME = output.map

# Test sources
TEST_SRCS += $(LIBS_SRCS)
TEST_NAME = test
TEST_OBJS := $(TEST_SRCS:.c=.o)

# Targets
###########################################################
# Default to compile project for default 'make'
all: build

# Compile all objects to executable
$(BIN_DIR)/$(OUTPUT_NAME): $(addprefix $(BUILD_DIR)/, $(OBJS))
	@$(MKDIR_P) $(BIN_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	$(SIZE) $@

# Invoke dump on project output
.PHONY: dump
dump: $(BIN_DIR)/$(OUTPUT_NAME)
	$(DUMP) $^

# remaps executable to the bin folder
.PHONY: $(OUTPUT_NAME)
$(OUTPUT_NAME): $(BIN_DIR)/$(OUTPUT_NAME)
	@echo Output will be in bin folder

# Compile individual objects from source files with dependency file
$(BUILD_DIR)/%.o: %.c
	@$(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) $(DFLAGS) -o $(BUILD_DIR)/$*.d $<
	$(CC) $(CFLAGS) -c $< -o $@

# Remaps an individual object file to the correct folder
.PHONY: %.o
%.o: $(BUILD_DIR)/%.o
	@echo Output will be in build folder

# Preprocess individual source files
$(BUILD_DIR)/%.i: %.c
	@$(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -E $< -o $@

# Remaps an individual preprocessed output to the correct folder
.PHONY: %.i
%.i: $(BUILD_DIR)/%.i
	@echo Output will be in build folder

# Assemble individual source files
$(BUILD_DIR)/%.s: %.c
	@$(MKDIR_P) $(BUILD_DIR)
	$(CC) $(CFLAGS) -S $< -o $@

# Remaps an individual assembly file to the correct folder
.PHONY: %.s
%.s: $(BUILD_DIR)/%.s
	@echo Output will be in build folder

# Build all files and link
.PHONY: build
build: $(BIN_DIR)/$(OUTPUT_NAME)

# Preprocess all files
.PHONY: preprocess
preprocess: $(addprefix $(BUILD_DIR)/, $(PRES))

# Generate assembly output of all files
.PHONY: asm-file
asm-file: $(addprefix $(BUILD_DIR)/, $(ASSEMS))

# Compile all files but do not link
.PHONY: compile-all
compile-all: $(addprefix $(BUILD_DIR)/, $(OBJS))

# Upload to Beagle Bone Black via SCP
.PHONY: upload
upload: $(BIN_DIR)/$(OUTPUT_NAME)
	@echo Uploading project to BBB with SCP
	$(XFER) -r $(BIN_DIR)/$(OUTPUT_NAME) $(HOST_ADDR)

# Build library from library sources
.PHONY: build_lib
build-lib: $(BUILD_DIR)/$(LIB_NAME)

$(BUILD_DIR)/$(LIB_NAME): $(addprefix $(BUILD_DIR)/, $(LIB_OBJS))
	$(AR) rcs $@ $^

# Deletes build files and executables
.PHONY: clean
clean:
	@$(RM_F) $(BUILD_DIR)/*
	@$(RM_F) $(BIN_DIR)/*
	@echo Successfully cleaned.

# Build and execute test
.PHONY: test
test: $(addprefix $(BUILD_DIR)/, $(TEST_OBJS))
	@$(MKDIR_P) $(BIN_DIR)
	$(CC) $(CFLAGS) -o $(BIN_DIR)/$@ $^
	@echo Running unit tests...
	@$(BIN_DIR)/$(TEST_NAME)
