COINES_INSTALL_PATH ?= submodules/coines
EXAMPLE_FILE ?= bhy2cli.c
BHY_INTF ?= 
COINES_BACKEND ?= COINES_BRIDGE

CLI360_API_LOCATION ?= submodules/bhi360/source
CLI385_API_LOCATION ?= submodules/bhi385/source

API_LOCATION ?= source
BHY_DEF_LOCATION ?= $(API_LOCATION)/bhy_def
COMMON_LOCATION ?= source/common

VALID_LOCATION := FLASH
VALID_TARGET := PC MCU_APP30 MCU_APP31

ifeq ($(TARGET),$(filter $(TARGET),MCU_APP30 MCU_APP31))
LOCATION := FLASH
endif

# Automatically collect all .c files in CLI_API_LOCATION
CLI360_API_SRCS := $(wildcard $(CLI360_API_LOCATION)/*.c)
CLI385_API_SRCS := $(wildcard $(CLI385_API_LOCATION)/*.c)

API_C_SRCS := $(filter-out source/$(EXAMPLE_FILE), $(wildcard $(API_LOCATION)/*.c))

C_SRCS += \
$(API_C_SRCS) \
$(COMMON_LOCATION)/common.c \
$(API_LOCATION)/callbacks/bhy2cli_callbacks.c \
$(API_LOCATION)/callbacks/common_callbacks.c \
$(BHY_DEF_LOCATION)/bhy_defs.c \
$(CLI360_API_SRCS) \
$(CLI385_API_SRCS)

INCLUDEPATHS += . \
$(COMMON_LOCATION) \
$(API_LOCATION)/callbacks \
$(API_LOCATION) \
$(BHY_DEF_LOCATION) \
$(FW_LOCATION)/firmware \
$(CLI360_API_LOCATION) \
$(CLI385_API_LOCATION) \

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY_INTF), I2C)
CFLAGS += -DBHY_USE_I2C
endif


ifneq ($(filter $(TARGET), $(VALID_TARGET)), $(TARGET))
$(error TARGET is not valid; please specify TARGET as one of: $(VALID_TARGET))
endif

# Check if LOCATION is valid
ifneq ($(filter $(LOCATION), $(VALID_LOCATION)), $(LOCATION))
$(error LOCATION is not valid; please specify LOCATION as FLASH)
endif

include $(COINES_INSTALL_PATH)/coines.mk
