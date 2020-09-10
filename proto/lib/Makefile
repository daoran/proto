include ../config.mk

SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.cpp=.o))
HPPS = $(wildcard *.hpp)
GCHS = $(addprefix $(BLD_DIR)/, $(HPPS:.hpp=.hpp.gch))
LIB = $(BLD_DIR)/libproto.a

PCH = $(BLD_DIR)/proto.hpp.gch

default: $(PCH) $(LIB)

$(PCH): proto.hpp
	$(COMPILE_HEADER)

$(BLD_DIR)/%.o: %.cpp %.hpp
	$(COMPILE_OBJECT)

$(BLD_DIR)/%.hpp.gch : %.hpp
	$(COMPILE_HEADER)

$(BLD_DIR)/libproto.a: $(OBJS)
	$(MAKE_STATIC_LIB)
