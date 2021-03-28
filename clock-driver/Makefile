OPTION = -O3
CXX = c++
CXXFLAGS = $(OPTION) -Wall -I/opt/vc/include/
LDFLAGS = -L/opt/vc/lib/
DEST = /usr/local/bin/
LIBS = -lbcm_host -lpthread
OBJS = clock-driver.o
PROGRAM = clock-driver
SERVICE = $(PROGRAM).service

all: $(PROGRAM)

$(PROGRAM): Makefile $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $(PROGRAM)

$(OBJS): Makefile

clean:
	rm -f *.o *~ $(PROGRAM)

install: $(PROGRAM) uninstall
	install -s $(PROGRAM) $(DEST)
	cp $(SERVICE) /etc/systemd/system/
	systemctl enable $(SERVICE)

uninstall:
	systemctl disable $(SERVICE) || :
	systemctl stop $(SERVICE) || :
	rm /etc/systemd/system/$(SERVICE) || :
	rm $(DEST)$(PROGRAM) || :
