CXX=g++
CXXFLAGS=\
	-std=c++11 \
	-Wall \
	-Winvalid-pch \
	-O3 \
	-fPIC \
	-I$(PWD)/lib \
	-I$(BLD_DIR) \
	-I/usr/include/eigen3 \
	-I/usr/include/yaml-cpp \
	`pkg-config --cflags opencv`
LIBS=\
	-L$(BLD_DIR) -lproto \
	`pkg-config --libs opencv` \
	-lyaml-cpp \
	-lpthread

BLD_DIR = $(PWD)/build

AR = ar
AR_FLAGS = -rvs

COMPILE_OBJECT = \
	@echo "CXX [$<]"; \
	$(CXX) $(CXXFLAGS) -c $< -o $@

COMPILE_HEADER = \
	@echo "PCH [$<]"; \
	$(CXX) $(CXXFLAGS) -c $< -o $@

MAKE_STATIC_LIB = \
	@echo "AR [libproto.a]"; \
	$(AR) $(AR_FLAGS) $@ $^

MAKE_TEST = \
	@echo "TEST [$<]"; \
	$(CXX) $< -o $@ $(LIBS) $(CXXFLAGS)
