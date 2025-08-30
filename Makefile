# Makefile for the graphical aircraft finder application
# It links against system libraries for libcurl, cJSON, SDL2, SDL2_ttf, and SDL2_mixer.
# It also automatically embeds the font file into the executable.

CC = gcc
TARGET = find_closest_plane
SRCS = main.c
OBJS = $(SRCS:.c=.o)

# Get compiler and linker flags from pkg-config
SDL_CFLAGS := $(shell pkg-config --cflags sdl2 SDL2_ttf SDL2_mixer)
SDL_LDFLAGS := $(shell pkg-config --libs sdl2 SDL2_ttf SDL2_mixer)

# Add all flags together
CFLAGS = -Wall -Wextra -O2 -g $(SDL_CFLAGS)
LDFLAGS = -lcurl -lcjson -lm $(SDL_LDFLAGS)

.PHONY: all clean

all: font_data.h $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

# Rule to convert the font file into a C header file
font_data.h: PressStart2P-Regular.ttf
	@echo "Embedding font..."
	@xxd -i PressStart2P-Regular.ttf > font_data.h

# Make sure the font header is generated before compiling main.c
main.o: font_data.h

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS) font_data.h

