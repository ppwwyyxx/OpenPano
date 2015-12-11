# $File: Makefile
# $Date: Wed Jun 17 20:52:38 2015 +0800

OBJ_DIR = obj
TARGET = image-stitching

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CXX ?= g++
	EIGEN_PATH ?= /usr/include/eigen3
endif
ifeq ($(UNAME_S),Darwin)
	CXX = g++-5
	EIGEN_PATH ?= /usr/local/include/eigen3
endif
INCLUDE_DIR = -I. -isystem $(EIGEN_PATH) -isystem third-party

OPTFLAGS ?= -O3 -msse3 #-march=native
#OPTFLAGS ?= -g -O0
DEFINES = -DDEBUG  	# comment out this line improves speed

# optional extra packages
#LIBS = opencv
#INCLUDE_DIR += $(shell pkg-config --cflags $(LIBS))
#LDFLAGS += $(shell pkg-config $(LIBS) --libs)

# use mkl for eigen
ifdef MKLROOT
	INCLUDE_DIR += -I$(MKLROOT)/include
	LDFLAGS += -m64 -L$(MKLROOT)/lib/intel64 -L$(MKLROOT)/../compiler/lib/intel64
	LDFLAGS += -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_intel_thread -lmkl_core -liomp5 -lpthread -lm
	DEFINES += -DEIGEN_USE_MKL_ALL
endif

CXXFLAGS = $(INCLUDE_DIR) -fopenmp -Wall -Wextra
CXXFLAGS += $(DEFINES) -std=c++11 $(OPTFLAGS)

LDFLAGS += $(OPTFLAGS)
LDFLAGS += -ljpeg -fopenmp

SHELL = bash
ccSOURCES = $(shell find . -name "*.cc" | sed 's/^\.\///g')
OBJS = $(addprefix $(OBJ_DIR)/,$(ccSOURCES:.cc=.o))
DEPFILES = $(OBJS:.o=.d)

.PHONY: all clean run

all: $(TARGET)

ifneq ($(MAKECMDGOALS), clean)
sinclude $(DEPFILES)
endif

$(OBJ_DIR)/%.o: %.cc
	@echo "[cc] $< ..."
	@$(CXX) -c $< -o $@ $(CXXFLAGS)

$(OBJ_DIR)/%.d: %.cc  Makefile
	@mkdir -p $(dir $@)
	@echo "[dep] $< ..."
	@$(CXX) $(CXXFLAGS) $(DEFINES) -MM -MT "$(OBJ_DIR)/$(<:.cc=.o) $(OBJ_DIR)/$(<:.cc=.d)" "$<"  > "$@"


$(TARGET): $(OBJS)
	@echo "Linking ..."
	@$(CXX) $(OBJS) -o $@ $(LDFLAGS)
	@echo "done."

clean:
	@rm -rvf $(OBJ_DIR)

