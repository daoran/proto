CXX=g++
CFLAGS=\
	-Wall \
	-I/usr/include/eigen3 \
	-I/usr/include/yaml-cpp \
	`pkg-config --cflags opencv`
LIBS=\
	-lyaml-cpp \
	`pkg-config --libs opencv`

BLD_DIR = $(PWD)/build

AR = ar
AR_FLAGS = -rvs

SRCS = $(wildcard lib/*.cpp)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.cpp=.o))

COMPILE_OBJ = \
	@echo "CXX [%<]"; \
	$(CXX) $(CFLAGS) -c $< -o $@

MAKE_STATIC_LIB = \
	@echo "AR [libproto.a]"; \
	$(AR) $(AR_FLAGS) $@ $^

default:
	@echo $(SRCS)
	echo $(OBJS)

$(BLD_DIR)/%.o : lib/%.cpp lib/%.hpp
	$(COMPILE_OBJ)

$(BLD_DIR)/libproto.a : $(OBJS)
	echo $(OBJS)
	$(MAKE_STATIC_LIB)
