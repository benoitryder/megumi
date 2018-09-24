CXX = g++
CPPFLAGS += -Wall -Wextra -Werror
# sadly, it does warn about designated initializers
# (despite what is stated in docs)
CPPFLAGS += -Wno-missing-field-initializers
#CPPFLAGS += -DSPDLOG_FMT_EXTERNAL
CXXFLAGS += -std=c++17 -O3 -flto
LDFLAGS += -lboost_program_options$(BOOST_SUFFIX) -lfmt
DISABLE_LIB ?=

NAME = megumi
SRCS = $(wildcard $(addsuffix /*.cpp,. model block))
OBJS = $(SRCS:.cpp=.o)
LIB_SRCS = $(filter-out ./main.cpp,$(SRCS))
LIB_OBJS = $(LIB_SRCS:.cpp=.o)

ifeq ($(OS),Windows_NT)
LDFLAGS += -lws2_32
EXE_LDFLAGS += -Wl,--enable-auto-import
LIB_LDFLAGS += -Wl,--out-implib,$(TARGET_LIB).a -Wl,--export-all-symbols
LIB_EXT = .dll
EXE_EXT = .exe
else
EXE_EXT =
LIB_EXT = .so
endif

ifneq ($(PREFIX),)
CPPFLAGS += -I$(PREFIX)/include
LDFLAGS += -L$(PREFIX)/lib
endif

TARGET_EXE = $(NAME)$(EXE_EXT)
ifeq ($(DISABLE_LIB),)
TARGET_LIB = $(NAME)$(LIB_EXT)
TARGET_EXE_OBJS = $(TARGET_LIB)
CXXFLAGS += -fPIC
else
TARGET_LIBS =
TARGET_EXE_OBJS = $(LIB_OBJS)
endif

.PHONY: default all clean

default: $(TARGET_EXE)

all: $(TARGET_LIB) $(TARGET_EXE)

-include $(OBJS:.o=.d)

$(TARGET_EXE): $(TARGET_EXE_OBJS) main.o
	$(CXX) main.o $(TARGET_EXE_OBJS) -o $@ $(LDFLAGS) $(EXE_LDFLAGS)

ifeq ($(DISABLE_LIB),)
$(TARGET_LIB): $(LIB_OBJS)
	$(CXX) $(LIB_OBJS) -o $@ -shared $(LDFLAGS) $(LIB_LDFLAGS)
endif

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -c $< -o $@


clean:
	rm -f $(TARGET) $(OBJS) $(OBJS:.o=.d)


