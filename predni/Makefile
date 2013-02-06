##########################################
# Makefile for voltage monitor project   #
# (c) Kamil Rezac 2011                   #
##########################################

# target file prefix (main source is $(TRG).c, output $(TRG).hex
	TRG = ocelac_predni

# command definitions
	CC	= /usr/local/CrossPack-AVR/bin/avr-gcc -DFRONT -DCURRENT_MODES=6 -DMAX=900
	AS	= /usr/local/CrossPack-AVR/bin/avr-gcc -x assembler-with-cpp	
	RM	= rm -f
	RN	= mv
	CP	= cp
	BIN	= /usr/local/CrossPack-AVR/bin/avr-objcopy
	SIZE	= /usr/local/CrossPack-AVR/bin/avr-size
	INCDIR	= .

# output format can be srec, ihex (avrobj is always created)
	FORMAT = ihex

# MCU type
	MCU = attiny461

# list of source files, add application-specific files here
	SRC= $(TRG).c ../common.c

# additional directory with includes
	INC	= ..

# compiler flags
	CPFLAGS	= -g -Os -Wall -Wstrict-prototypes -I$(INC) -Wa,-ahlms=$(<:.c=.lst) -mmcu=$(MCU)

# linker flags
	LDFLAGS = -Wl,-Map=$(TRG).map,--cref -mmcu=$(MCU)

# list of object file (no need to modify)
	OBJ	= $(SRC:.c=.o)

#main build target (build starts here)
all:	$(TRG).elf $(TRG).hex $(TRG).ok

# generic object file build rule for c source file
%.o : %.c 
	$(CC) -c $(CPFLAGS) -I$(INCDIR) $< -o $@

# generic assembly file build rule for c source file
%.s : %.c
	$(CC) -S $(CPFLAGS) -I$(INCDIR) $< -o $@

# link rule to create elf file form object files
%.elf: $(OBJ)
	$(CC) $(OBJ) $(LIB) $(LDFLAGS) -o $@

# rule for creating output file (for programming into device)
%.hex: %.elf
	$(BIN) -O $(FORMAT) -R .eeprom $< $@

# all target passed, display program size
%ok:
	$(SIZE) -C --mcu=$(MCU) $(TRG).elf
	@echo "Errors: none"

# clean target (remove all files created during build
clean:
	$(RM) $(OBJ)
	$(RM) $(SRC:.c=.s)
	$(RM) $(SRC:.c=.lst)
	$(RM) $(TRG).map
	$(RM) $(TRG).elf
	$(RM) $(TRG).cof
	$(RM) $(TRG).obj
	$(RM) $(TRG).a90
	$(RM) $(TRG).hex
	$(RM) $(TRG).sym
	$(RM) $(TRG).eep
	$(RM) $(TRG).hex
	$(RM) *.bak
	$(RM) *.log
	@echo "Errors: none"

 
# include makefile for common robbus dependencies, modify according to placement

$(TRG).o: $(TRG).c

prog: all
	avrdude -p t461 -c usbasp -U flash:w:$(TRG).hex

prog_fuses:
	avrdude -p t461 -c usbasp -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

read:
	avrdude -p t461 -c usbasp -U eeprom:r:eeprom.epp:h
	cat eeprom.epp

.PHONY: prog all read