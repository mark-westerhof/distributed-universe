CC := g++
SRCEXT := cpp

INCLUDES := -I/usr/local/include -Ishared
LFLAGS := -L/usr/local/lib
CFLAGS := -Wall -O3 -std=c++0x
LIBS := -lpthread

#Shared
SHARED_SRCDIR := shared
SHARED_SOURCES := $(shell find $(SHARED_SRCDIR) -type f -name *.$(SRCEXT))

#Master
MASTER_TARGET := master
MASTER_SRCDIR := master_src
MASTER_CFLAGS := $(CFLAGS)
MASTER_LIBS := $(LIBS) -lGL -lGLU -lglfw
MASTER_SOURCES := $(SHARED_SOURCES) $(shell find $(MASTER_SRCDIR) -type f -name *.$(SRCEXT))
MASTER_OBJS := $(MASTER_SOURCES:.$(SRCEXT)=.o)
MASTER_DEPS := $(MASTER_OBJS:.o=.deps)

#Worker
WORKER_TARGET := worker
WORKER_SRCDIR := worker_src
WORKER_CFLAGS := $(CFLAGS)
WORKER_LIBS := $(LIBS)
WORKER_SOURCES := $(SHARED_SOURCES) $(shell find $(WORKER_SRCDIR) -type f -name *.$(SRCEXT))
WORKER_OBJS := $(WORKER_SOURCES:.$(SRCEXT)=.o)
WORKER_DEPS := $(WORKER_OBJS:.o=.deps)

all: $(MASTER_TARGET) $(WORKER_TARGET)

#Master
$(MASTER_TARGET): $(MASTER_OBJS)
	@echo "Linking master..."; $(CC) $^ -o $(MASTER_TARGET) $(LFLAGS) $(MASTER_LIBS)
		 
$(MASTER_SRCDIR)/%.o: $(MASTER_SRCDIR)/%.$(SRCEXT)
	@echo "  CC $<"; $(CC) $(INCLUDES) $(CFLAGS) -MD -MF $(@:.o=.deps) -c -o $@ $<

#Worker
$(WORKER_TARGET): $(WORKER_OBJS)
	@echo "Linking worker..."; $(CC) $^ -o $(WORKER_TARGET) $(LFLAGS) $(WORKER_LIBS)
		 
$(WORKER_SRCDIR)/%.o: $(WORKER_SRCDIR)/%.$(SRCEXT)
	@echo "  CC $<"; $(CC) $(INCLUDES) $(CFLAGS) -MD -MF $(@:.o=.deps) -c -o $@ $<
	
#Shared
$(SHARED_SRCDIR)/%.o: $(SHARED_SRCDIR)/%.$(SRCEXT)
	@echo "  CC $<"; $(CC) $(INCLUDES) $(CFLAGS) -MD -MF $(@:.o=.deps) -c -o $@ $<

#Clean
clean: cleanmaster cleanworker

cleanmaster:
	@echo "Cleaning master..."; $(RM) $(MASTER_OBJS) $(MASTER_DEPS) $(MASTER_TARGET)

cleanworker:
	@echo "Cleaning worker..."; $(RM) $(WORKER_OBJS) $(WORKER_DEPS) $(WORKER_TARGET)

-include $(MASTER_DEPS)
-include $(WORKER_DEPS)

.PHONY: cleanmaster
.PHONY: cleanworker
