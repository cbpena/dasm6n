--------------------------------------------------------------
DASM6N (v0.90.0)
6502 (NES) disassembler
--------------------------------------------------------------

A 6502 disassembler specifically for the NES. Has a basis in loopy's
asm6 assembler and uses CDL data from a custom version of Mesen.

This is free software.  You may use, modify, and / or redistribute any part
of this software in any fashion.

--------------------------------------------------------------
Command line
--------------------------------------------------------------

Usage:

		dasm6n [-options] romfile cdlfile [labelfile]

Options:
		
		-?				show help
		-S				split each bank into it's own *.asm file
		-E				expand BIT instruction when a JMP/JSR/branch targets the operand
		-R				use PPU+APU+I/O register names from nesdev wiki
		-B <filename>	read bank size configuration
		-W<16>			how many bytes of data to output on one line
		-U				print percentages of UNUSED rom data
		-D				print out debug info to "debug.txt"

--------------------------------------------------------------
Bank Size Configuration
--------------------------------------------------------------

Example:

	1F
	8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 16

The first line denotes how many logical banks are present (in hexidecimal). The
next line specifies the size of each bank. In the example there are 31 banks,
the first 30 are 8K and the last is 16K. A 'v' can be appended to the end of
the bank size to indicated that bank has CPU vectors in it.

--------------------------------------------------------------
Label File
--------------------------------------------------------------

A list of labels to substitute for addresses.

Format:

	<bank#> <address> <label>

Examples:

	-1 0042 zero_page_label
	-1 03C0 ram_label
	-1 6000 wram_label
	 0 8000 prg_bank0_label
	1F C800 prg_bank1F_label

--------------------------------------------------------------
<EOF>