CC := gcc
CXX := g++
CFLAGS:= -Wall -O3 -march=native -fopenmp -pg
INCS := -I/usr/include/eigen3
LIBS := \
	../../deps/OpenBLAS/libopenblas.a \
	-lm -lpthread -lgfortran -lm

.PHONY: all dirs
all: dirs bench_matmul
# all: dirs bench_matmul bench_svd-jacobi
# all: dirs bench_svd-lapacke

dirs:
	@mkdir -p bin

clean:
	@rm -rf bin

bench_matmul-eigen: bench_matmul-eigen.cpp
	@echo "CXX [$<]"; $(CXX) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_matmul-blas: bench_matmul-blas.c
	@echo "CC [$<]"; $(CC) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_matmul-handcode: bench_matmul-handcode.c
	@echo "CC [$<]"; $(CC) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_matmul: bench_matmul-eigen bench_matmul-blas bench_matmul-handcode

bench_svd-eigen: bench_svd-eigen.cpp
	@echo "CXX [$<]"; $(CXX) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_svd-lapacke: bench_svd-lapacke.c
	@echo "CC [$<]"; $(CC) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_svd-jacobi: bench_svd-jacobi.c
	@echo "CC [$<]"; $(CC) $(CFLAGS) $< -o bin/$@ $(INCS) $(LIBS)

bench_svd: bench_svd-eigen bench_svd-lapacke bench_svd-jacobi
