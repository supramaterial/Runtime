#
# Copyright (c) 2013-2014 ELL-i co-operative.
#
# ELL-i software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ELL-i software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ELL-i software.  If not, see <http://www.gnu.org/licenses/>.
#
#

TOP = $(shell cd ..; pwd)/

TARGETS = \
    robot_library \
    test_pinMode \
    test_digitalRead \
    test_digitalWrite \
    test_analogWrite \
    test_Serial_begin \
    test_Serial_write_uint8 \
    test_SPI_begin \
    test_SPI_transfer \

.PHONY : $(TARGETS) all clean tests

all:	$(TARGETS)
all:	TARGET=all

clean:	$(TARGETS)
clean:	TARGET=clean

$(TARGETS): %:
	@echo
	@echo "========================================================="
	@echo "==== $*"
	@echo "========================================================="
	@if [ -f $*/Makefile ]; then \
		$(MAKE) -C $* PLATFORM=$(PLATFORM) $(TARGET); \
	else \
		$(MAKE) -C $* -f $(TOP)make/app.mk \
			APPLICATION_SRC_DIR=$(PWD)/$* \
			TOP=$(TOP) APP=$@ PLATFORM=$(PLATFORM) $(TARGET); \
	fi

tests:	PLATFORM = emulator
tests:  $(TARGETS)
	for t in $(TARGETS); do DYLD_BIND_AT_LAUNCH=1 ./$$t/$$t; done
