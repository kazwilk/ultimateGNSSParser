PROG_NAME        := GNSS_parser

CPPFLAGS         := -Wall -Wpedantic -Walloc-zero -Warray-bounds -Wbool-compare -Wpointer-arith  -Wno-pointer-compare -Wsizeof-pointer-memaccess -Wswitch-default
CXX              := g++

#SRCS             := ../../src/ultimateGNSSParser.cpp linuxGNSS.cpp
#OBJS             := ${SRCS:.cpp=.o}
OBJS             := ultimateGNSSParser.o linuxGNSS.o

PROG_INCLUDE_DIR :=../../src

CPPFLAGS         += $(foreach includedir,$(PROG_INCLUDE_DIR),-I$(includedir))

.PHONY: all

all: $(PROG_NAME)

ultimateGNSSParser.o : ../../src/ultimateGNSSParser.h
	$(CXX) $(CPPFLAGS) -c ../../src/ultimateGNSSParser.cpp -o ultimateGNSSParser.o

$(PROG_NAME): $(OBJS)
	$(CXX) $(CPPFLAGS) $(OBJS) -o $(PROG_NAME)

clean:
	rm -rf *.o
	rm -rf ../../src/*.o
	rm -rf GNSS_parser
