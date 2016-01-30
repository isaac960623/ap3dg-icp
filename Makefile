# inspiration: http://hiltmon.com/blog/2013/07/03/a-simple-c-plus-plus-project-structure/
# TODO: Move `libmongoclient.a` to /usr/local/lib so this can work on production servers - The TODO at the top reminds me that I am using a different version of a library in development and it must be removed before deployment
#
CC := g++ # This is the main compiler
# CC := clang --analyze # and comment out the linker last line for sanity
SRCDIR := src
BUILDDIR := build
TARGET := bin/runner
# Final Paths
INSTALLBINDIR := /usr/local/bin

# The TARGET is the main executable of the project, in this case bin/runner. Type make and this is what gets built

# The SOURCES list is dynamic, I don’t want to manually have to maintain this list as I program. Anything in the src folder will be included in the compile as long as it has a SRCEXT extension

SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
CFLAGS := -g # -Wall

# Folder Lists
# Note: Intentionally excludes the root of the include folder so the lists are clean
# include/**/*
INCDIRS := $(shell find include -name '*.h' -exec dirname {} \; | sort | uniq)
INCLIST := $(patsubst include/%,-I include/%,$(INCDIRS))
BUILDLIST := $(patsubst include/%,$(BUILDDIR)/%,$(INCDIRS))

# Shared Compiler Flags
# CFLAGS := -c
INC := -I include $(INCLIST) -I /usr/local/include -I /usr/local/Cellar/glfw3/3.1.2/include -I /opt/local/include/ -I /Users/ucaHome/libraries/ann_1.1.2/include
LIB := -L lib -L /usr/local/lib -L /usr/local/Cellar/glfw3/3.1.2/lib -L /opt/local/lib -L /Users/ucaHome/libraries/ann_1.1.2/lib -lstdc++ -lOpenMeshTools -lOpenMeshCore -lANN
FRAMEWORK = -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

# The OBJECTS list is also dynamic and uses a Makefile trick to build the list based on available sources

$(TARGET): $(OBJECTS)
	@echo " Linking..."
	$(CC) $^ -o $(TARGET) $(LIB) $(FRAMEWORK)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo " Building..."
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	$(RM) -r $(BUILDDIR) $(TARGET)

install:
	@echo "Installing $(EXECUTABLE)..."
	cp $(TARGET) $(INSTALLBINDIR)

distclean:
	@echo "Removing $(EXECUTABLE)"
	rm $(INSTALLBINDIR)/$(EXECUTABLE)

# Tests
# tester:
# $(CC) $(CFLAGS) test/tester.cpp $(INC) $(LIB) -o bin/tester

# Spikes
# ticket:
#  $(CC) $(CFLAGS) spikes/ticket.cpp $(INC) $(LIB) -o bin/ticket

# The .PHONY clean is brilliant, it nukes the build folder and the main executable. It does not clean spike or test executables though
.PHONY: clean
