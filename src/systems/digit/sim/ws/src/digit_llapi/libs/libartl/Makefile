# Build for linux using bundled zstd by default
PLATFORM := LINUX # Set to WIN for a windows build (requires mingw-w64)
USE_SYSTEM_ZSTD := # Set to 1 to use system zstd library

# Compilation settings
CFLAGS  := -O2 -Wall -Wextra
INC     :=
LIBS    :=

# Output library names
LDLIBOUT := libartl.so
LSLIBOUT := libartl.a
WDLIBOUT := artl.dll
WSLIBOUT := artl.lib

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
CFLAGS  += -D__USE_MINGW_ANSI_STDIO=1
LIBS    += -Wl,-Bstatic -lpthread -Wl,-Bdynamic -lws2_32
DLIBOUT := $(WDLIBOUT)
SLIBOUT := $(WSLIBOUT)
else
CC      := gcc
CFLAGS  += -fPIC
DLIBOUT := $(LDLIBOUT)
SLIBOUT := $(LSLIBOUT)
endif

# Find all .c files used to create the library
SRC := $(wildcard *.c)

# Choose either dynamically-linked system zstd or statically-linked zstd
# Dynamic produces a smaller binary, static is more self-contained
ifdef USE_SYSTEM_ZSTD
LIBS += -lzstd
else
INC += -Izstd
SRC += zstd/zstd.c
endif

# Library object files
OBJ := $(patsubst %.c,%.o,$(SRC))

# Tests
TESTS := file_test file_partial_test pubsub_test unstructured_test

# By default, build both the static and dynamic library
all: $(DLIBOUT) $(SLIBOUT)

# Object files
%.o: %.c
	$(CC) -c $< $(INC) $(CFLAGS) -o $@

# Dynamic library
$(DLIBOUT): $(OBJ)
	$(CC) $(OBJ) -shared $(LIBS) -o $(DLIBOUT)
	strip $(DLIBOUT)

# Static library
$(SLIBOUT): $(OBJ)
	ar rcs $(SLIBOUT) $(OBJ)

# Clean just object files
cleanobj:
	find . -name '*.o' -delete

# Clean all generated files
clean: cleanobj
	rm -f $(LDLIBOUT)
	rm -f $(WDLIBOUT)
	rm -f $(LSLIBOUT)
	rm -f $(WSLIBOUT)
	rm -f $(TESTS)
	rm -f test.log
	rm -f unstructured.log

# Build and run all of the tests/examples
# Note: Tests are linux-only
test: $(TESTS)

$(TESTS): %: tests/%.c $(DLIBOUT) Makefile
	$(CC) $< $(DLIBOUT) -g -I. -lpthread -Wl,--rpath='$$ORIGIN' -o $@
	./$@

# Virtual targets
.PHONY: all clean cleanobj test
