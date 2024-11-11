UNAME_S = $(shell uname -s)
WORK_DIR  = $(shell pwd)
BUILD_DIR = $(WORK_DIR)/build
MAKE = make

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
END='\033[0m'

SERIAL_DIR= $(shell find ./3rdparty -name "src-serial")
THIRD_PARTY_LIB_DIR= $(shell find ./3rdparty -name "lib")

CC = g++
CPPFLAGS = -std=c++20 -O0 -g
#CPPFLAGS += -Wall
CPPFLAGS += -I$(WORK_DIR)/include

# NOTE: turn on debug here
CPPFLAGS += -D__DEBUG__

# FIXME: imtui dependency linking
CPPFLAGS += -I "./3rdparty/include"
CPPFLAGS += -L "./3rdparty/lib"

LDFLAGS += -lm -lpthread -ldl -lrt -lserial

#LDFLAGS = `pkg-config sdl --libs`

SRC = $(wildcard src/*.cc) $(wildcard src/**/*.cc)
OBJ = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(SRC))))
BIN = rx78-2

.PHONY: all clean

all: dirs $(BIN)

dirs:
	@echo -e + $(BLUE)MKDIR$(END) $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)

# Setup can0 can1 and serial interfase
os-deps:
	@sudo ip link set can0 up type can bitrate 1000000
	@echo -e + $(BLUE)CAN0$(END) $(GREEN)UP$(END)
	@sudo ip link set can1 up type can bitrate 1000000
	@echo -e + $(BLUE)CAN1$(END) $(GREEN)UP$(END)
	@sudo chmod -R 777 /dev/ttyACM0
	@echo -e + $(BLUE)ACM0$(END) $(GREEN)UP$(END)

run: all
	$(BUILD_DIR)/$(BIN)

clean-build: clean
	@make all -j8

mini-pc:
	@sudo docker exec --workdir /home/zzlinus/dev/cpp/NeoRMControl_OneForALL fc54835e2a55 make clean-build
	sshpass -p 1 scp build/rx78-2 gkd@192.168.0.114:/home/gkd/dev
	sshpass -p 1 ssh gkd@192.168.0.114 "/home/gkd/dev/rx78-2"

mini-pc-os-deps:
	sshpass -p 1 ssh root@192.168.0.114 "ip link set CAN_LEFT_HEAD up type can bitrate 1000000"
	sshpass -p 1 ssh root@192.168.0.114 "ip link set CAN_RIGHT_HEAD up type can bitrate 1000000"
	sshpass -p 1 ssh root@192.168.0.114 "ip link set CAN_CHASSIS up type can bitrate 1000000"
	sshpass -p 1 ssh root@192.168.0.114 "chmod 666 /dev/IMU_LEFT"
	sshpass -p 1 ssh root@192.168.0.114 "chmod 666 /dev/IMU_RIGHT"
	sshpass -p 1 ssh root@192.168.0.114 "chmod 666 /dev/IMU_BIG_YAW"

serial: $(SERIAL_DIR)
	@$(MAKE) -C $< -j8
	@echo -e + $(BLUE)MV$(END) $(SERIAL_DIR)/build/libserial.a $(THIRD_PARTY_LIB_DIR)
	@mv $(SERIAL_DIR)/build/libserial.a $(THIRD_PARTY_LIB_DIR)

$(BIN): $(OBJ) serial
	@echo -e + $(GREEN)LN$(END) $(BUILD_DIR)/$(BIN)
	@$(CC) -o $(BUILD_DIR)/$(BIN) $(OBJ) $(CPPFLAGS) $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cc
	@mkdir -p $(dir $@) 
	@echo -e + $(GREEN)CC$(END) $<
	@$(CC) -o $@ -c $< $(CPPFLAGS)

clean-serial: $(SERIAL_DIR)
	$(MAKE) -C $< clean

clean: clean-serial
	@echo -e + $(BLUE)RM$(END) 3rdparty/lib/libserial.a
	@rm 3rdparty/lib/libserial.a
	@echo -e + $(BLUE)RM$(END) $(BUILD_DIR)/$(BIN) OBJs
	@rm -rf $(BUILD_DIR)/$(BIN) $(OBJ)
