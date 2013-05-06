# $File: Makefile
# $Date: Sun May 05 23:03:22 2013 +0800

OBJ_DIR = obj
TARGET = main

INCLUDE_DIR = -Iinclude
DEFINES = -DDEBUG

#OPTFLAGS = -O3 -g -Wall -Wextra -Wcpp -O
OPTFLAGS = -O3

LIBS =
#INCLUDE_DIR += $(shell pkg-config --cflags $(LIBS))

CXXFLAGS = $(INCLUDE_DIR) -fopenmp
CXXFLAGS += $(DEFINES) -std=c++11 $(OPTFLAGS)
CXXFLAGS += $(shell Magick++-config --cxxflags)

LDFLAGS = -fopenmp $(OPTFLAGS)
#LDFLAGS += $(shell pkg-config $(LIBS) --libs)
LDFLAGS += $(shell Magick++-config --ldflags --libs)

CC = g++
SHELL = bash
ccSOURCES = $(shell find -name "*.cc" | sed 's/^\.\///g')
OBJS = $(addprefix $(OBJ_DIR)/,$(ccSOURCES:.cc=.o))
DEPFILES = $(OBJS:.o=.d)

.PHONY: all clean run

all: $(TARGET)

sinclude $(DEPFILES)

$(OBJ_DIR)/%.o: %.cc
	@echo "[cc] $< ..."
	@$(CC) -c $< -o $@ $(CXXFLAGS)

$(OBJ_DIR)/%.d: %.cc  Makefile
	@mkdir -p $(dir $@)
	@echo "[dep] $< ..."
	@$(CC) $(INCLUDE_DIR) $(DEFINES) -MM -MT "$(OBJ_DIR)/$(<:.cc=.o) $(OBJ_DIR)/$(<:.cc=.d)" "$<"  > "$@"


$(TARGET): $(OBJS)
	@echo "Linking ..."
	@$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo "done."


clean:
	@rm -rf $(OBJ_DIR)

run: $(TARGET)
	./$(TARGET) lenna.png lenna_r.png

