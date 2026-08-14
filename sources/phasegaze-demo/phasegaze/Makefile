# Makefile for CSI Sweep - 3D Hemisphere Visualization
#
# Build with: make
# Clean with: make clean
# Debug build: make DEBUG=1

CC = gcc

CFLAGS = -O3 -flto -Wall -Wextra -Wno-unused-parameter
LDFLAGS = -flto
LIBS = -lfftw3f -lglfw -lGL -lSDL2 -lSDL2_ttf -lm -lpthread

ifdef DEBUG
CFLAGS = -g -O0 -Wall -Wextra -Wno-unused-parameter -DDEBUG
LDFLAGS =
endif

SRCS = main.c utils.c ring_buffer.c signal_processing.c worker.c \
       sphere_render.c camera.c control_window.c widgets.c settings.c \
       imu_worker.c bno080.c mask.c snapshot.c shutter_stream.c \
       external_controls.c phase_cal.c tx_slots.c
OBJS = $(SRCS:.c=.o)

TARGET = csi_sweep

HEADERS = config.h fpga_csi.h utils.h ring_buffer.h signal_processing.h worker.h \
          sphere_render.h camera.h control_window.h widgets.h settings.h \
          imu_worker.h bno080.h mask.h snapshot.h shutter_stream.h \
          external_controls.h phase_cal.h tx_slots.h

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

main.o: main.c config.h fpga_csi.h utils.h worker.h signal_processing.h sphere_render.h camera.h control_window.h imu_worker.h snapshot.h shutter_stream.h external_controls.h phase_cal.h
utils.o: utils.c utils.h
ring_buffer.o: ring_buffer.c ring_buffer.h utils.h config.h fpga_csi.h
signal_processing.o: signal_processing.c signal_processing.h config.h fpga_csi.h
worker.o: worker.c worker.h config.h utils.h ring_buffer.h signal_processing.h mask.h phase_cal.h imu_worker.h
mask.o: mask.c mask.h config.h signal_processing.h
sphere_render.o: sphere_render.c sphere_render.h config.h worker.h shutter_stream.h
camera.o: camera.c camera.h
control_window.o: control_window.c control_window.h config.h widgets.h worker.h settings.h imu_worker.h external_controls.h phase_cal.h
external_controls.o: external_controls.c external_controls.h config.h
widgets.o: widgets.c widgets.h config.h
settings.o: settings.c settings.h
imu_worker.o: imu_worker.c imu_worker.h bno080.h
bno080.o: bno080.c bno080.h
snapshot.o: snapshot.c snapshot.h sphere_render.h config.h
shutter_stream.o: shutter_stream.c shutter_stream.h snapshot.h sphere_render.h config.h
phase_cal.o: phase_cal.c phase_cal.h
tx_slots.o: tx_slots.c tx_slots.h
