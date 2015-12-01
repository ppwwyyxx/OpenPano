# $File: Makefile
# $Date: Wed Jun 17 20:52:38 2015 +0800

OBJ_DIR = obj
TARGET = image-stitching
CXX ?= g++

EIGEN_PATH ?= /usr/include/eigen3
INCLUDE_DIR = -I. -isystem $(EIGEN_PATH) -isystem third-party
DEFINES = -DDEBUG

OPTFLAGS ?= -O3 -msse3 #-march=native
#OPTFLAGS = -g -O0

#LIBS =
#INCLUDE_DIR += $(shell pkg-config --cflags $(LIBS))

CXXFLAGS = $(INCLUDE_DIR) -fopenmp -Wall -Wextra
CXXFLAGS += $(DEFINES) -std=c++1y $(OPTFLAGS)

LDFLAGS = -fopenmp $(OPTFLAGS)
#LDFLAGS += $(shell pkg-config $(LIBS) --libs)
LDFLAGS += -ljpeg

SHELL = bash
ccSOURCES = $(shell find -name "*.cc" | sed 's/^\.\///g')
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

