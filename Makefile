CC = clang
CFLAGS = -Wall -Wextra -O2 -I./firmware/include
SRC = firmware/src/bms_core.c firmware/src/soc_estimator.c firmware/src/cell_balancer.c firmware/src/protection.c
TEST_SRC = tests/test_bms.c

all: test

test:
	$(CC) $(CFLAGS) $(SRC) $(TEST_SRC) -o run_tests -lm
	./run_tests

sim:
	python3 simulation/simulate_bms.py

clean:
	rm -f run_tests
