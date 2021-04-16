include ../config.mk

SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.cpp=.o))
LIB = $(BLD_DIR)/libproto.a

default: $(LIB)

$(BLD_DIR)/%.o: %.cpp %.hpp
	$(COMPILE_OBJECT)

$(BLD_DIR)/libproto.a: $(OBJS)
	$(MAKE_STATIC_LIB)
