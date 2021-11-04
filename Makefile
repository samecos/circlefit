THE_OS := $(shell uname -s)

default:
	@echo "Detected OS: ${THE_OS}"
	$(MAKE) CC=gcc CXX=g++ \
		CXXFLAGS='$(CXXFLAGS) -D_AFXEXT -DGLOG_NO_ABBREVIATED_SEVERITIES -Wall -Wextra -Wno-deprecated-copy -pipe -O3 -g -ffast-math -flto -std=c++17 -DNDEBUG' \
		LDFLAGS='$(LDFLAGS) -flto -g' \
		run

debug:
	@echo "Detected OS: ${THE_OS}"
	$(MAKE) CC=gcc CXX=g++ \
		CXXFLAGS='$(CXXFLAGS) -D_AFXEXT -DGLOG_NO_ABBREVIATED_SEVERITIES -Wall -Wextra -Wno-deprecated-copy -pipe -Og -g -std=c++17' \
		LDFLAGS='$(LDFLAGS) -g' \
		run

clang:
	@echo "Detected OS: ${THE_OS}"
	$(MAKE) CC=clang CXX=clang++ \
		CXXFLAGS='$(CXXFLAGS) -D_AFXEXT -DGLOG_NO_ABBREVIATED_SEVERITIES -Wall -Wextra -Wno-deprecated-copy -O3 -ffast-math -flto -std=c++17 -DNDEBUG' \
		LDFLAGS='$(LDFLAGS) -flto -fuse-linker-plugin' \
		run

DYNAMIC_LIBS = -lgsl -lgslcblas
CXXFLAGS += -I/opt/homebrew/include
CXXFLAGS += -I./
LDFLAGS += -L/opt/homebrew/lib
CPPFLAGS += -MD -MP

sources = circlefitsolver.cpp main.cpp

objects = $(sources:.cpp=.o)
deps = $(sources:%.cpp=%.d)

-include $(deps)
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

run: $(objects)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS) $(DYNAMIC_LIBS)

clean:
	-$(RM) run $(objects) $(deps)

.PHONY: clean default debug clang