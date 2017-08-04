# $File: Makefile
# $Date: Wed Jun 17 20:52:38 2015 +0800

OBJ_DIR = obj
TARGET = image-stitching

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CXX ?= g++
	EIGEN3_INCLUDE_DIR ?= /usr/include/eigen3
endif
ifeq ($(UNAME_S),Darwin)
	CXX ?= g++
	EIGEN3_INCLUDE_DIR ?= /usr/local/include/eigen3
endif
INCLUDE_DIR = -I. -isystem $(EIGEN3_INCLUDE_DIR) -isystem third-party

ifneq (,$(findstring clang,$(CXX)))
	OMP_FLAG=-fopenmp=libomp
else
	OMP_FLAG=-fopenmp
endif

OPTFLAGS ?= -O3 -msse3 -march=native
#OPTFLAGS ?= -g3 -fsanitize=address,undefined -O2 -lasan
#OPTFLAGS ?= -g3 -fsanitize=leak -O2 -lubsan
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

CXXFLAGS ?=
CXXFLAGS += $(INCLUDE_DIR) $(OMP_FLAG) -Wall -Wextra -Wnon-virtual-dtor
CXXFLAGS += $(DEFINES) -std=c++11 $(OPTFLAGS)

LDFLAGS += $(OPTFLAGS)
LDFLAGS += -ljpeg $(OMP_FLAG)

SOFLAGS = $(shell python-config --includes --ldflags)

SHELL = bash
# sources to include
SRCDIRS = lib feature stitch third-party/lodepng
ccSOURCES = $(shell find $(SRCDIRS) -name "*.cc" | sed 's/^\.\///g')
OBJS = $(addprefix $(OBJ_DIR)/,$(ccSOURCES:.cc=.o))
DEPFILES = $(OBJS:.o=.d)

SO = python/openpano.so

.PHONY: all clean run

all: $(TARGET)
py: $(SO)

ifneq ($(MAKECMDGOALS), clean)
sinclude $(DEPFILES)
endif

$(OBJ_DIR)/%.o: %.cc
	@echo "[cc] $< ..."
	@$(CXX) -c $< -o $@ $(CXXFLAGS)

$(OBJ_DIR)/%.d: %.cc Makefile
	@mkdir -pv $(dir $@)
	@echo "[dep] $< ..."
	@$(CXX) $(CXXFLAGS) -MM -MT "$(OBJ_DIR)/$(<:.cc=.o) $(OBJ_DIR)/$(<:.cc=.d)" "$<"  > "$@"

$(TARGET): main.cc $(OBJS)
	@echo "Linking ..."
	@$(CXX) $^ -o $@ $(CXXFLAGS) $(LDFLAGS)
	@echo "done."

$(SO): $(OBJS) python/pybind.cc
	@echo "[so] $@ ..."
	@$(CXX) $^ -fPIC -shared -o $@ $(CXXFLAGS) $(LDFLAGS) $(SOFLAGS)
	@echo "done."

clean:
	@rm -rvf $(OBJ_DIR)

