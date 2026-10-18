CC = gcc
CFLAGS = -O2 -Wall -I include
LDFLAGS = -lm

SRC = src/failure.c src/main.c
OBJ = $(SRC:.c=.o)
EXEC = failure_sim

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f src/*.o $(EXEC) *.csv *.png

run: $(EXEC)
	./$(EXEC)

benchmark:
	python3 plot.py

.PHONY: all clean run benchmark

