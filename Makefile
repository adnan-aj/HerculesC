CC = gcc
CFLAGS = -g -Wall

TARGET = herc
OBJS = xbox-hercules.o evtest.o herc-comm.o
DEPS =

all: $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS)

clean:
	rm -f *.o $(TARGET)
