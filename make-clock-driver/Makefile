OPTION = -O3
CXX = c++
CXXFLAGS = $(OPTION) -Wall -I/opt/vc/include
LDFLAGS = -L/opt/vc/lib
DEST = /usr/local/bin
LIBS = -lbcm_host -lpthread
OBJS = clock-driver.o
PROGRAM = clock-driver

all: $(PROGRAM)

$(PROGRAM): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $(PROGRAM)

clean:
	rm -f *.o *~ $(PROGRAM)

install: $(PROGRAM)
	install -s $(PROGRAM) $(DEST)
