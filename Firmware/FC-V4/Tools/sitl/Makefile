# SITL Monte Carlo harness: the flight core (Core/hyades) + a 6DOF plant.
# Same warning set as the firmware for the flight code; -O2 for speed.
CORE := ../../Core/hyades
INC  := -I$(CORE)/math -I$(CORE)/gnc -I$(CORE)/mission -I$(CORE)/svc -I../../Core/Inc
SRC  := sitl.c $(CORE)/mission/flight.c $(CORE)/mission/phase.c $(CORE)/mission/fdir.c \
        $(CORE)/mission/detect.c $(CORE)/gnc/control.c $(CORE)/gnc/iekf.c $(CORE)/gnc/nav.c
sitl: $(SRC) ../../Core/Inc/board.h
	$(CC) -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter -D_DEFAULT_SOURCE $(INC) -o $@ $(SRC) -lm
clean:
	rm -f sitl
