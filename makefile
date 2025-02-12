# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# enable debug metadata
CFLAGS += -g
CXX_FLAGS += -g

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

# Core repo cpp and c files
SRC_C += $(wildcard core/src/*.cpp)
SRC_C += $(wildcard core/src/*.c)
SRC_C += $(wildcard core/src/*/*.cpp)
SRC_C += $(wildcard core/src/*/*.c)
SRC_C += $(wildcard core/src/*/*/*.c)
SRC_C += $(wildcard core/src/*/*/*.cpp)
SRC_C += $(wildcard core/src/*/*/*/*.c)
SRC_C += $(wildcard core/src/*/*/*/*.cpp)


OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)
SRC_H  = $(wildcard include/*/*.h)
SRC_H  = $(wildcard include/*/*/*.h)

# Core repo header files
SRC_H += $(wildcard core/include/*.h)
SRC_H += $(wildcard core/include/*/*.h)
SRC_H += $(wildcard core/include/*/*/*.h)
SRC_H += $(wildcard core/include/*/*/*/*.h)

# Vendor include directories
INC += -Ivendor/eigen


# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk