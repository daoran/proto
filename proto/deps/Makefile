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

dirs:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p log
	@mkdir -p share
	@mkdir -p src

# openblas: dirs
# 	@echo "Building openblas ..."
# 	@bash openblas.bash > log/openblas_build.log 2>&1

# suitesparse: dirs openblas
# 	@echo "Building suitesparse ..."
# 	@bash suitesparse.bash > log/suitesparse_build.log 2>&1

# stb: dirs
# 	@echo "Building stb ..."
# 	@bash stb.bash > log/stb_build.log 2>&1
#
# fast: dirs
# 	@echo "Building fast ..."
# 	@bash fast.bash > log/fast_build.log 2>&1
#
# glfw: dirs
# 	@echo "Building glfw ..."
# 	@bash glfw.bash > log/glfw_build.log 2>&1

openblas: dirs
	@echo "Building openblas ..."
	@bash openblas.bash

stb: dirs
	@echo "Building stb ..."
	@bash stb.bash

fast: dirs
	@echo "Building fast ..."
	@bash fast.bash

glfw: dirs
	@echo "Building glfw ..."
	@bash glfw.bash
