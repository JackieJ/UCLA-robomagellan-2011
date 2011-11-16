UNAME := $(shell uname)

all: ax3500

install:
	cd build && make install

uninstall:
	cd build && make uninstall

ax3500:
	@mkdir -p build
	@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

clean:
	-cd build && make clean
	rm -rf build bin lib doc/html

.PHONY: doc
doc:
	@doxygen doc/Doxyfile
ifeq ($(UNAME),Darwin)
	@open doc/html/index.html
endif

.PHONY: test
test:
	@mkdir -p build
	@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) -DAX3500_BUILD_TESTS=1 ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif
	cd bin && ./ax3500_tests
