INCLUDES=-I/usr/include/eigen3
LIBS=-lyaml-cpp -lapriltags -lprototype
CC=mkoctfile --mex
SRCS=$(wildcard *.cpp)
OBJS=$(SRCS:.cpp=.o)

all: $(OBJS)

%.o: %.cpp
	@echo "Compiling [$<]"
	@$(CC) $< $(INCLUDES) $(LIBS)

clean:
	@echo "Cleaning *.mex *.o"
	@rm -f *.mex
	@rm -f *.o
