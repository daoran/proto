#!/bin/bash
set -e
# make clean
make

# ldd ./bin/bench_matmul-blas
# ldd ./bin/bench_svd-lapacke
# ldd ./bin/bench_svd-lapacke

# ./bin/bench_matmul-blas
# ./bin/bench_matmul-eigen
# ./bin/bench_svd-eigen
# ./bin/bench_svd-lapacke
# time ./bin/bench_svd-jacobi 100
