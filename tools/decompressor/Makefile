COINES_INSTALL_PATH ?= ../../submodules/coines

EXAMPLE_FILE ?= decompressor.c

INCLUDEPATHS += ../../source
INCLUDEPATHS += ../../source/bhy_def

# Override other target definitions
VALID_TARGET := PC

ifneq ($(filter $(TARGET), $(VALID_TARGET)), $(TARGET))
$(error TARGET is not valid; please specify TARGET as one of: $(VALID_TARGET))
endif

include $(COINES_INSTALL_PATH)/coines.mk
