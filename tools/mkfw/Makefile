
HEADERS = crc32.h
SOURCES = main.c crc32.c
BIN = mkfw

FLAGS_COMMON = -Wall -Werror -Wextra -std=c11
FLAGS_RELEASE = -O3 $(FLAGS_COMMON)
FLAGS_DEBUG = -Og -ggdb3 $(FLAGS_COMMON)

all: release

.PHONY: release
release: $(BIN)

$(BIN): $(SOURCES) $(HEADERS) Makefile
	$(CC) $(FLAGS_RELEASE) $(SOURCES) -o $@ 

.PHONY: debug
debug: $(BIN).dbg

$(BIN).dbg: $(SOURCES) $(HEADERS) Makefile
	$(CC) $(FLAGS_DEBUG) $(SOURCES) -o $@ 

.PHONY: clean
clean:
	rm -f $(BIN) $(BIN).dbg
