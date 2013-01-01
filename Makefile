CXX = g++
CPPFLAGS += -g -Wall -Wextra -Werror
# sadly, it does warn about designated initializers
# (despite what is stated in docs)
CPPFLAGS += -Wno-missing-field-initializers
CXXFLAGS += -std=c++11 -O2
LDFLAGS += -lboost_program_options$(BOOST_SUFFIX)

TARGET = megumi
SRCS = $(wildcard $(addsuffix /*.cpp,. model block))
OBJS = $(SRCS:.cpp=.o)

ifeq ($(OS),Windows_NT)
LDFLAGS +=  -lws2_32
LDFLAGS += -static-libgcc -static-libstdc++
endif

ifneq ($(PREFIX),)
CPPFLAGS += -I$(PREFIX)/include
LDFLAGS += -L$(PREFIX)/lib
endif


.PHONY: default all clean

default: $(TARGET)

all: default

-include $(OBJS:.o=.d)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -c $< -o $@


clean:
	rm -f $(TARGET) $(OBJS) $(OBJS:.o=.d)


