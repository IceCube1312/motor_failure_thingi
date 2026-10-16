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
	rm -f src/*.o $(EXEC) flight_stock.csv flight_relaxed.csv benchmark_body_rates.png benchmark_altitude.png benchmark_summary.png

run: $(EXEC)
	./$(EXEC)

.PHONY: all clean run
