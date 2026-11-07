# Host unit tests for the portable core (Core/hyades). Pure C, no HAL.
#   make            build and run every test (non-zero exit on any failure)
#   make tools      check Tools/fc_pull.py decodes firmware-packed records,
#                   and test_tools.py (fc_link demux/parsers + simulator end to end)
#   make clean
# Same warning set as the firmware (software design guidelines E1).
CC      ?= gcc
CORE    := ../Core/hyades
CFLAGS  := -std=c11 -O2 -g -Wall -Wextra -Werror -Wundef -Wshadow \
           -Wdouble-promotion -Wformat=2 -Wno-unused-parameter -fno-common \
           -DUNITY_INCLUDE_DOUBLE \
           -Iunity -I$(CORE)/math -I$(CORE)/gnc -I$(CORE)/mission -I$(CORE)/svc -I../Core/Inc
OUT     := ../build-host

TESTS := test_crc test_nmea test_logrec test_phase test_fdir test_control test_iekf test_nav test_detect

SRC_test_crc     := $(CORE)/svc/crc.c
SRC_test_nmea    := $(CORE)/svc/nmea.c
SRC_test_logrec  := $(CORE)/svc/logrec.c $(CORE)/svc/crc.c
SRC_test_phase   := $(CORE)/mission/phase.c
SRC_test_fdir    := $(CORE)/mission/fdir.c
SRC_test_control := $(CORE)/gnc/control.c
SRC_test_iekf    := $(CORE)/gnc/iekf.c
SRC_test_nav     := $(CORE)/gnc/nav.c
SRC_test_detect  := $(CORE)/mission/detect.c $(CORE)/mission/phase.c

all: $(addprefix $(OUT)/,$(TESTS))
	@fail=0; for t in $(TESTS); do $(OUT)/$$t || fail=1; done; exit $$fail

.SECONDEXPANSION:
$(OUT)/test_%: test_%.c $(OUT)/unity.o $$(SRC_test_%) | $(OUT)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Third-party: built with the default warnings only (not our code, not -Werror).
$(OUT)/unity.o: unity/unity.c | $(OUT)
	$(CC) -std=c11 -O2 -g -DUNITY_INCLUDE_DOUBLE -Iunity -c -o $@ $<

$(OUT):
	mkdir -p $(OUT)

# Firmware-packed log -> Python decoder (Tools/fc_pull.py) round trip.
tools: $(OUT)/gen_log_fixture
	$(OUT)/gen_log_fixture $(OUT)/fixture.bin
	python3 tools_check.py $(OUT)/fixture.bin
	python3 test_tools.py

$(OUT)/gen_log_fixture: gen_log_fixture.c $(CORE)/svc/logrec.c $(CORE)/svc/crc.c | $(OUT)
	$(CC) $(CFLAGS) -o $@ $^ -lm

clean:
	rm -rf $(OUT)

.PHONY: all tools clean
