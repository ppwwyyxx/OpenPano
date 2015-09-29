# $File: Makefile
# $Date: Wed Jun 17 20:52:38 2015 +0800

OBJ_DIR = obj
TARGET = image-stitching

INCLUDE_DIR = -Iinclude
DEFINES = -DDEBUG

OPTFLAGS = -O3 -march=native
OPTFLAGS = -g -O0

LIBS = libpng
INCLUDE_DIR += $(shell pkg-config --cflags $(LIBS))

CXXFLAGS = $(INCLUDE_DIR) -fopenmp -Wall -Wextra
CXXFLAGS += $(DEFINES) -std=c++11 $(OPTFLAGS)

LDFLAGS = -fopenmp $(OPTFLAGS)
LDFLAGS += $(shell pkg-config $(LIBS) --libs)
LDFLAGS += -lX11 -ljpeg

CC = g++
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
	@$(CC) -c $< -o $@ $(CXXFLAGS)

$(OBJ_DIR)/%.d: %.cc  Makefile
	@mkdir -p $(dir $@)
	@echo "[dep] $< ..."
	@$(CC) $(CXXFLAGS) $(DEFINES) -MM -MT "$(OBJ_DIR)/$(<:.cc=.o) $(OBJ_DIR)/$(<:.cc=.d)" "$<"  > "$@"


$(TARGET): $(OBJS)
	@echo "Linking ..."
	@$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo "done."


clean:
	@rm -rvf $(OBJ_DIR)

run: $(TARGET)
	./$(TARGET) lenna.png lenna_r.png

