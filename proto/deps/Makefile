# default: dirs openblas suitesparse stb fast glfw
# .PHONY: dirs openblas suitesparse stb fast glfw

default: dirs stb fast glfw
.PHONY: dirs stb fast glfw

clean:
	@rm -rf bin
	@rm -rf include
	@rm -rf lib
	@rm -rf log
	@rm -rf share
	@rm -rf src
	@rm *.log

dirs:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

openblas: dirs
	@echo "[Building OpenBLAS]"
	@bash openblas.bash > log/openblas_build.log 2>&1
	@echo "[Finished Building OpenBLAS!]"

suitesparse: dirs openblas
	@echo "[Building SuiteSparse]"
	@bash suitesparse.bash > log/suitesparse_build.log 2>&1
	@echo "[Finished Building SuiteSparse!]"

stb: dirs
	@echo "[Building STB]"
	@bash stb.bash > log/stb_build.log 2>&1
	@echo "[Finished Building STB!]"

fast: dirs
	@echo "[Building FAST]"
	@bash fast.bash > log/fast_build.log 2>&1
	@echo "[Finished Building FAST!]"

glfw: dirs
	@echo "[Building GLFW]"
	@bash glfw.bash > log/glfw_build.log 2>&1
	@echo "[Finished Building GLFW!]"
