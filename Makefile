CXX = g++
CPPFLAGS = -g -Wall -Wextra -Werror
# sadly, it does warn about designated initializers
# (despite what is stated in docs)
CPPFLAGS += -Wno-missing-field-initializers
CXXFLAGS = -std=c++11
LDFLAGS = -lboost_program_options

TARGET = megumi
SRCS = $(wildcard $(addsuffix /*.cpp,. model block))
OBJS = $(SRCS:.cpp=.o)


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


