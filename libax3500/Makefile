UNAME := $(shell uname)

all: ax3500

install:
	cd build && make install

uninstall:
	cd build && make uninstall

ax3500:
	@mkdir -p build
	#@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

clean:
	-cd build && make clean
	rm -rf build bin lib doc/html
	# Clean Eclipse build folders too
	rm -rf Debug Release
