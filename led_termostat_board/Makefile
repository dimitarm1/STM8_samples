NAME=testproj
SDCC=sdcc
HEX2BIN=hex2bin
ifeq ($(OS),Windows_NT)
  CCFLAGS=-DSTM8S003 -I$(CURDIR)/.. -IC:/ProgramFiles/SDCC/include -mstm8 --out-fmt-elf --debug
else
  CCFLAGS=-DSTM8S003 -I../ -I/usr/share/sdcc/include -mstm8 --out-fmt-elf --debug
endif
LDFLAGS= -lstm8 -mstm8 --out-fmt-elf --debug
FLASHFLAGS=-cstlinkv2 -pstm8s003

SRC=$(wildcard *.c)
# ATTENTION: FIRST in list should be file with main()
OBJ=$(SRC:%.c=%.rel)
TRASH=$(OBJ) $(SRC:%.c=%.rst) $(SRC:%.c=%.asm) $(SRC:%.c=%.lst)
TRASH+=$(SRC:%.c=%.sym) $(NAME).elf $(NAME).lk $(NAME).map
INDEPENDENT_HEADERS=$(CURDIR)../stm8l.h Makefile

all: $(NAME).elf

$(SRC) : %.c : %.h $(INDEPENDENT_HEADERS)
#	@touch $@
#	@echo $@

%.h: ;

clean:
	rm -f $(TRASH)

load: $(NAME).bin
	stm8flash $(FLASHFLAGS) -wf $(NAME).bin

%.rel: %.c
	$(SDCC) $(CCFLAGS) -c $<

$(NAME).elf: $(OBJ)
	$(SDCC) $(LDFLAGS) $(OBJ) -o $(NAME).elf	

.PHONY: all
