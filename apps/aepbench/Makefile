# Targets
TARGETS := aepbench.bin

# Source files required to build the target
CFILES   := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/*.c))

# Libraries required to build the target
LIBS-y							= muslc sel4 sel4muslcsys sel4vka sel4allocman sel4platsupport \
                                  sel4vspace sel4utils sel4simple sel4simple-default platsupport \
                                  sel4bench cpio utils
LIBS = $(LIBS-y)

include $(SEL4_COMMON)/common.mk
