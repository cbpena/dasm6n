
/*  dasm6n History:
0.90.0
	* Initial Version
*/



#include <direct.h>

#if defined(_WIN32)
#define mkdir(dir, mode) _mkdir(dir)
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>
#include <stdarg.h>
//#include <unistd.h>


#define VERSION "0.90.0"


/*
For PRG ROM:

	fPdjAADC

	       C = Whether it was accessed as code.
	       D = Whether it was accessed as data.
	       AA = Into which ROM bank it was mapped when last accessed:
	               00 = $8000-$9FFF        01 = $A000-$BFFF
	               10 = $C000-$DFFF        11 = $E000-$FFFF
	       j = Whether address is the target of a jump instruction
	       d = Whether indirectly accessed as data.
	               (e.g. as the destination of an LDA ($nn),Y instruction)
	       P = If logged as PCM audio data.
	       f = Whether the byte is the start of function (sub)
*/


// ================================================================================

typedef unsigned char byte;

typedef struct
{
	byte prg;
	byte chr;
	byte map;
	byte mir;
	byte sram;
} ines_header;

typedef struct
{
	const char *name;
	byte bank;
	long address;
} label;

typedef struct
{
	long start;				// start of bank
	long size;				// size of bank
	bool vectors;			// contains vectors
} bank;

const byte CODE = 0x01;
const byte DATA = 0x02;
const byte BANK = 0x0C;
const byte JUMP = 0x10;
// IND = 0x20;
const byte PCM  = 0x40;
const byte SUB  = 0x80;

const unsigned int INITLISTSIZE = 128;	//initial label list size



// ================================================================================
//		GLOBALS
// ================================================================================

const char *errmsg;

// flags
int flag_splitprg=0;		// -S				: split each bank into it's own *.asm file
int flag_expandbit=0;		// -E				: expand BIT instruction when a JMP/JSR/branch targets the operand
int flag_regname=0;			// -R				: use PPU+APU+I/O register names from nesdev wiki
int flag_bankcfg=0;			// -B <filename>	: read bank size configuration
int flag_datawidth=16;		// -W<16>			: how many bytes of data to output on one line
int flag_unused=0;			// -U				: print percentages of UNUSED rom data
int flag_debug=0;			// -D				: print out debug info to "debug.txt"

// data
long romfilesize=0;			// size of rom data
char *romfilename=0;		// rom filename
byte *romfile=nullptr;		// rom data

// cdl
long cdlfilesize=0;			// size of cdl data
char *cdlfilename=0;		// cdl filename
byte *cdlfile=nullptr;		// cdl data
byte *cdl8000=nullptr;
byte *cdlA000=nullptr;
byte *cdlC000=nullptr;
byte *cdlE000=nullptr;
byte *cdlIndexLB=nullptr;
byte *cdlIndexUB=nullptr;
const unsigned int EXT_CDL_FIELDS = 6;

// banks
char *bankfilename=0;				// filename for bank cfg (if provided)
int   logicalbankcount=0;			// how many banks (can be a mix of 8K, 16K, 24K, or 32K)
bank *logicalbank=nullptr;			// starting address/length/vectors of a logical bank
int   physicalbankcount=0;			// how many 8K banks (according to the ines header)
byte *physicalbankindex=nullptr;	// logical bank # of a physical bank

// labels
label **labellist;
int labelcount;				//# of labels in labellist
int labelmax;				//max # of labels labellist can hold
char *labelfilename=0;		// filename for label list

// dissassembler
ines_header header;

byte bank_id;
long base_addr, prev_base_addr;
byte opcode;
byte addr_mode;

long unusedcount=0;





// ================================================================================
//		ADDRESSING MODES
// ================================================================================
enum  opaddr           {ACC,    IMM,    IND,    INDX,   INDY,   ZPX,    ZPY,    ABSX,   ABSY,   ZP,     ABS,    REL,    IMP,    ZPL,    ZPXL,   ZPYL	};
int   opsize[]       = {0,      1,      2,      1,      1,      1,      1,      2,      2,      1,      2,      1,      0,      2,      2,      2		};
const char *ophead[] = {"",     "#",    "(",    "(",    "(",    "",     "",     "",     "",     "",     "",     "",     "",     "!",    "!",    "!"		};
const char *optail[] = {"",     "",     ")",    ", X)", "), Y", ", X",  ", Y",  ", X",  ", Y",  "",     "",     "",     "",     "",     ", X",  ", Y"	};





// ================================================================================
//		6502 OPCODES
// ================================================================================
enum opcodes {
	ADC, AND, ASL, 
	BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC, BVS,
	CLC, CLD, CLI, CLV, CMP, CPX, CPY,
	DEC, DEX, DEY,
	EOR,
	INC, INX, INY,
	JMP, JSR,
	LDA, LDX, LDY, LSR,
	NOP,
	ORA,
	PHA, PHP, PLA, PLP,
	ROL, ROR, RTI, RTS,
	SBC, SEC, SED, SEI, STA, STX, STY,
	TAX, TAY, TSX, TXA, TXS, TYA,
	
	_I_
};

const char *opmnem[]= {
	"ADC", "AND", "ASL",
	"BCC", "BCS", "BEQ", "BIT", "BMI", "BNE", "BPL", "BRK", "BVC", "BVS",
	"CLC", "CLD", "CLI", "CLV", "CMP", "CPX", "CPY",
	"DEC", "DEX", "DEY",
	"EOR",
	"INC", "INX", "INY",
	"JMP", "JSR",
	"LDA", "LDX", "LDY", "LSR",
	"NOP",
	"ORA",
	"PHA", "PHP", "PLA", "PLP",
	"ROL", "ROR", "RTI", "RTS",
	"SBC", "SEC", "SED", "SEI", "STA", "STX", "STY",
	"TAX", "TAY", "TSX", "TXA", "TXS", "TYA",
	
	"Illegal opcode"
};

byte matrix_opc[] =
{
//  x0    x1    x2    x3    x4    x5    x6    x7    x8    x9    xA    xB    xC    xD    xE    xF
	BRK,  ORA,  _I_,  _I_,  _I_,  ORA,  ASL,  _I_,  PHP,  ORA,  ASL,  _I_,  _I_,  ORA,  ASL,  _I_,		// 0x
	BPL,  ORA,  _I_,  _I_,  _I_,  ORA,  ASL,  _I_,  CLC,  ORA,  _I_,  _I_,  _I_,  ORA,  ASL,  _I_,		// 1x
	JSR,  AND,  _I_,  _I_,  BIT,  AND,  ROL,  _I_,  PLP,  AND,  ROL,  _I_,  BIT,  AND,  ROL,  _I_,		// 2x
	BMI,  AND,  _I_,  _I_,  _I_,  AND,  ROL,  _I_,  SEC,  AND,  _I_,  _I_,  _I_,  AND,  ROL,  _I_,		// 3x
	RTI,  EOR,  _I_,  _I_,  _I_,  EOR,  LSR,  _I_,  PHA,  EOR,  LSR,  _I_,  JMP,  EOR,  LSR,  _I_,		// 4x
	BVC,  EOR,  _I_,  _I_,  _I_,  EOR,  LSR,  _I_,  CLI,  EOR,  _I_,  _I_,  _I_,  EOR,  LSR,  _I_,		// 5x
	RTS,  ADC,  _I_,  _I_,  _I_,  ADC,  ROR,  _I_,  PLA,  ADC,  ROR,  _I_,  JMP,  ADC,  ROR,  _I_,		// 6x
	BVS,  ADC,  _I_,  _I_,  _I_,  ADC,  ROR,  _I_,  SEI,  ADC,  _I_,  _I_,  _I_,  ADC,  ROR,  _I_,		// 7x
	_I_,  STA,  _I_,  _I_,  STY,  STA,  STX,  _I_,  DEY,  _I_,  TXA,  _I_,  STY,  STA,  STX,  _I_,		// 8x
	BCC,  STA,  _I_,  _I_,  STY,  STA,  STX,  _I_,  TYA,  STA,  TXS,  _I_,  _I_,  STA,  _I_,  _I_,		// 9x
	LDY,  LDA,  LDX,  _I_,  LDY,  LDA,  LDX,  _I_,  TAY,  LDA,  TAX,  _I_,  LDY,  LDA,  LDX,  _I_,		// Ax
	BCS,  LDA,  _I_,  _I_,  LDY,  LDA,  LDX,  _I_,  CLV,  LDA,  TSX,  _I_,  LDY,  LDA,  LDX,  _I_,		// Bx
	CPY,  CMP,  _I_,  _I_,  CPY,  CMP,  DEC,  _I_,  INY,  CMP,  DEX,  _I_,  CPY,  CMP,  DEC,  _I_,		// Cx
	BNE,  CMP,  _I_,  _I_,  _I_,  CMP,  DEC,  _I_,  CLD,  CMP,  _I_,  _I_,  _I_,  CMP,  DEC,  _I_,		// Dx
	CPX,  SBC,  _I_,  _I_,  CPX,  SBC,  INC,  _I_,  INX,  SBC,  NOP,  _I_,  CPX,  SBC,  INC,  _I_,		// Ex
	BEQ,  SBC,  _I_,  _I_,  _I_,  SBC,  INC,  _I_,  SED,  SBC,  _I_,  _I_,  _I_,  SBC,  INC,  _I_		// Fx
};

byte matrix_adr[] =
{
//  x0    x1    x2    x3    x4    x5    x6    x7    x8    x9    xA    xB    xC    xD    xE    xF
	IMP,  INDX, 0,    0,    0,    ZP,   ZP,   0,    IMP,  IMM,  ACC,  0,    0,    ABS,  ABS,  0,		// 0x
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0,		// 1x
	ABS,  INDX, 0,    0,    ZP,   ZP,   ZP,   0,    IMP,  IMM,  ACC,  0,    ABS,  ABS,  ABS,  0,		// 2x
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0,		// 3x
	IMP,  INDX, 0,    0,    0,    ZP,   ZP,   0,    IMP,  IMM,  ACC,  0,    ABS,  ABS,  ABS,  0,		// 4x
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0,		// 5x
	IMP,  INDX, 0,    0,    0,    ZP,   ZP,   0,    IMP,  IMM,  ACC,  0,    IND,  ABS,  ABS,  0,		// 6x
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0,		// 7x
	0,    INDX, 0,    0,    ZP,   ZP,   ZP,   0,    IMP,  0,    IMP,  0,    ABS,  ABS,  ABS,  0,		// 8x
	REL,  INDY, 0,    0,    ZPX,  ZPX,  ZPY,  0,    IMP,  ABSY, IMP,  0,    0,    ABSX, 0,    0,		// 9x
	IMM,  INDX, IMM,  0,    ZP,   ZP,   ZP,   0,    IMP,  IMM,  IMP,  0,    ABS,  ABS,  ABS,  0,		// Ax
	REL,  INDY, 0,    0,    ZPX,  ZPX,  ZPY,  0,    IMP,  ABSY, IMP,  0,    ABSX, ABSX, ABSY, 0,		// BF
	IMM,  INDX, 0,    0,    ZP,   ZP,   ZP,   0,    IMP,  IMM,  IMP,  0,    ABS,  ABS,  ABS,  0,		// Cx
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0,		// Dx
	IMM,  INDX, 0,    0,    ZP,   ZP,   ZP,   0,    IMP,  IMM,  IMP,  0,    ABS,  ABS,  ABS,  0,		// Ex
	REL,  INDY, 0,    0,    0,    ZPX,  ZPX,  0,    IMP,  ABSY, 0,    0,    0,    ABSX, ABSX, 0			// Fx
};



// ================================================================================
//		REGISTERS
// ================================================================================
const unsigned int ppu_reg_count = 8;
const unsigned int apu_reg_count = 22;

const char *ppu_reg[]= {
	"PPU_CTRL",
	"PPU_MASK",
	"PPU_STATUS",
	"OAM_ADDR",
	"OAM_DATA",
	"PPU_SCROLL",
	"PPU_ADDR",
	"PPU_DATA"
};

const int ppu_reg_addr[]= {
	0x2000,
	0x2001,
	0x2002,
	0x2003,
	0x2004,
	0x2005,
	0x2006,
	0x2007
};


const char *apu_reg[]= {
	"SQ1_VOL",
	"SQ1_SWEEP",
	"SQ1_LO",
	"SQ1_HI",
	
	"SQ2_VOL",
	"SQ2_SWEEP",
	"SQ2_LO",
	"SQ2_HI",
	
	"TRI_LINEAR",
	"TRI_LO",
	"TRI_HI",
	
	"NOISE_VOL",
	"NOISE_LO",
	"NOISE_HI",
	
	"DMC_FREQ",
	"DMC_RAW",
	"DMC_START",
	"DMC_LEN",
	
	"OAM_DMA",
	"SND_CHN",
	"JOY1",
	"JOY2",
};

const int apu_reg_addr[]= {
	0x4000,
	0x4001,
	0x4002,
	0x4003,
	
	0x4004,
	0x4005,
	0x4006,
	0x4007,
	
	0x4008,
	0x400A,
	0x400B,
	
	0x400C,
	0x400E,
	0x400F,
	
	0x4010,
	0x4011,
	0x4012,
	0x4013,
	
	0x4014,
	0x4015,
	0x4016,
	0x4017,
};


// ================================================================================
//		MESSAGES
// ================================================================================
char ReadError[]="Error reading file.";
char BadMapper[]="Unsupported mapper.";
char ExtCdlSize[]="Extended cdl not the correct size.";


void showhelp()
{
    puts("");
    puts("dasm6n " VERSION "\n");
    puts("Usage:  dasm6n [-options] romfile cdlfile [labelfile]\n");
    puts("    -?              show this help");
    puts("    -S              write prg banks into separate files");
    puts("    -E              force BIT expansion");
    puts("    -R              use PPU+APU+I/O register names");
    puts("    -B <filename>   read bank size configuration");
    puts("    -W<16>          set the data width for HEX directive");
    puts("    -U              print rom usage");
    puts("    -D              print debug info to debug.txt");
    puts("See README.TXT for more info.\n");
}





// ================================================================================
//		FUNCTIONS
// ================================================================================

// Prints printf-style message to stderr, then exits.
static void fatal_error( const char fmt [], ... )
{
    va_list args;
    
    va_start( args, fmt );
    fprintf( stderr, "\nError: " );
    vfprintf( stderr, fmt, args );
    fprintf( stderr, "\n\n" );
    va_end( args );
    
    exit( EXIT_FAILURE );
}

// Same as malloc(), but prints error and exits if allocation fails
static void* my_malloc( size_t s )
{
    void* p = malloc( s ? s : 1 );
    if ( p == NULL )
        fatal_error( "Out of memory" );
    
    return p;
}

// Same as common strdup(), but prints error and exits if allocation fails
static char* my_strdup(const char *in)
{
    size_t size = strlen( in ) + 1;
    char* out = static_cast<char*>( my_malloc(size) );
        memcpy( out, in, size );
    return out;
}


// ================================================================================
//		LABELS
// ================================================================================

// initialize label list
void initlabels()
{
	labellist	= (label**)my_malloc(INITLISTSIZE*sizeof(label*));
	labelcount	= 0;
	labelmax	= INITLISTSIZE;
}

// double list capacity
void growlist()
{
    label **tmp;
    
    labelmax <<= 1;
    tmp=(label**)my_malloc(labelmax * sizeof(label*));
    memcpy(tmp, labellist, labelcount  *sizeof(label*));
    free(labellist);
    labellist = tmp;
}

// check if label already defined
int findlabel(byte bank, long address)
{
	for (size_t i = 0; i < labelcount; i++)
	{
		if ( ((*(labellist[i])).bank == bank) && ((*(labellist[i])).address == address) )
			return i;
	}
	return -1;
}

int findlabelmirror(byte bank, long offset)
{
	long mask = (logicalbank[bank_id].size - 1);
	for (size_t i = 0; i < labelcount; i++)
	{
		if ( ((*(labellist[i])).bank == bank) && ( ((*(labellist[i])).address & mask) == (offset & mask)) )
			return i;
	}
	return -1;
}

// add label to list
void addlabel(byte bank, long address, char *name)
{
	label *p;
	char str[128];
	
	if (findlabel(bank, address) < 0)
	{
	    p = (label*)my_malloc(sizeof(label));
	    (*p).bank		= bank;
	    (*p).address	= address;
	    
	    if (name == 0)
	    {
	    	if (address < 0x0100)
	    		sprintf( str, "zp_%02X", address);
	    	else if (address < 0x2000)
				sprintf( str, "ram_%04X", address);
			else if ( (address >= 0x6000) && (address < 0x8000) )
				sprintf( str, "wram_%04X", address );
			else
				sprintf( str, "label_%02X_%04X", bank, address );
	    	
	    	(*p).name = my_strdup(str);
	    }
	    else
	    	(*p).name = my_strdup(name);
	    
		//fprintf( stdout, "label added = {%02X, %04X, \'%s\'}\n", (*p).bank, (*p).address, (*p).name );

		if (labelcount >= labelmax)
			growlist();

		labellist[labelcount] = p;
		labelcount++;

	}
}

int comparelabels(const void* arg1, const void* arg2)
{
	const label* a = *((label**)arg1);
	const label* b = *((label**)arg2);
	if(a->bank > b->bank) return 1;
	if(a->bank < b->bank) return -1;
	if(a->address > b->address) return 1;
	if(a->address < b->address) return -1;
	return 0;
}

// read labels from a list
void read_labels()
{
	FILE *f;
	int r;
	byte bank;
	long address;
	char name[256];
	
	if (labelfilename)
	{
		
		if( !(f = fopen(labelfilename, "r")) )
			fatal_error("Can't open file: %s", labelfilename);
		else
		{
			while( (r = fscanf(f, "%X %X %s", &bank, &address, name)) != EOF  )
				addlabel(bank, address, name);
		}
		
		 if(f) fclose(f);
		 
		 //for (int i = 0; i < labelcount; i++)
		 //		fprintf( stdout, "%s = %02X:%04X\n", labellist[i]->name, labellist[i]->bank, labellist[i]->address );
		 
	}
	else
		fprintf( stdout, "error reading label list\n" );
	
}


void add_registers_to_labels()
{
	for (int i = 0; i < ppu_reg_count; i++)
		addlabel( -1, ppu_reg_addr[i], (char*)ppu_reg[i] );

	for (int i = 0; i < apu_reg_count; i++)
		addlabel( -1, apu_reg_addr[i], (char*)apu_reg[i] );
}





// ================================================================================
//		BANKS
// ================================================================================

// read bank configuration
void read_bank_cfg()
{
	FILE *f;
	int r, size, lbi=0, pbi=0;
	long addr=0;
	unsigned char vec;
	
	if (bankfilename)
	{	
		if( !(f = fopen(bankfilename, "r")) )
			fatal_error("Can't open file: %s", bankfilename);
		else
		{
			if ( (r = fscanf(f, "%X", &logicalbankcount)) != EOF )
			{
				logicalbank = new bank[logicalbankcount];
				physicalbankindex = new byte[physicalbankcount];
				
				while ( (lbi < logicalbankcount) && (pbi < physicalbankcount) )
				{
					if ( (r = fscanf(f, "%d%c", &size, &vec)) == EOF)
						fatal_error("error reading bank cfg\n");

					size <<= 10;

					// set logical bank data
					logicalbank[lbi].start   = addr;
					logicalbank[lbi].size    = size;
					logicalbank[lbi].vectors = (vec == 'v');
					addr += size;

					while (size > 0)
					{
						physicalbankindex[pbi] = lbi;
						size -= 0x2000;
						pbi++;
					}
					lbi++;
				}
			}
			else
				fatal_error("error reading bank count\n");
		}
		
		 if(f) fclose(f);
	}
	else
		fatal_error("error reading bank sizes\n");
}

// set the number of banks to disassemble
void set_bank_count()
{
	long addr=0;
	
	if (flag_bankcfg)
		read_bank_cfg();
	else
	{
		physicalbankindex = new byte[physicalbankcount];
		
		switch (header.map)
		{
			case 0:	/* NROM */
				logicalbankcount = 1;
				logicalbank = new bank[logicalbankcount];
				
				logicalbank[0].start = 0;
				
				if (header.prg == 1)
					logicalbank[0].size = 0x4000;
				else
					logicalbank[0].size = 0x8000;
				
				logicalbank[0].vectors = true;
				
				for (int i = 0; i < physicalbankcount; i++)
					physicalbankindex[i] = 0;

				break;
				
			case 4:	/* MMC3 */
				logicalbankcount = physicalbankcount;
				logicalbank = new bank[logicalbankcount];
				
				for (int i = 0; i < physicalbankcount; i++)
				{
					logicalbank[i].start = addr;
					logicalbank[i].size = 0x2000;
					logicalbank[i].vectors = false;
					physicalbankindex[i] = i;
					addr += 0x2000;
				}
				
				logicalbank[physicalbankcount - 1].vectors = true;
				
				break;
			
			default:				errmsg = BadMapper;
		}
		
	}

}


// ================================================================================

// read binary file
byte* read_bin(char *filename, long& filesize) {
    FILE *f=0;
    byte* file;
	size_t result;

	if( !(f = fopen(filename,"rb")) )
		fatal_error("Can't open file: %s",filename);
	else
	{
		// get size
		fseek(f, 0, SEEK_END);
		filesize = ftell(f);
		// allocate memory
		file = new byte[filesize];
		if ( file != NULL)
		{
			// read rom file
			fseek(f, 0, SEEK_SET);
			result = fread(file, 1, filesize, f);
			
			if (result != filesize)
				errmsg=ReadError;
		}
	}
	
    if(f) fclose(f);
    
    return file;
}

byte* read_bin_block(FILE *f, long size)
{
	size_t result;
	byte* b = new byte[size];
	
	if ( b != NULL)
	{
		result = fread(b, 1, size, f);
		if (result != size)
		{
			errmsg=ReadError;
			delete[] b;
			b=NULL;
		}	
	}
	return b;
}

// read *.cdl.ext file
void read_extended_cdl(char *filename, long cdlsize)
{
	FILE *f=0;
	size_t result;
	long filesize;
	
	if( !(f = fopen(filename,"rb")) )
		fatal_error("Can't open file: %s",filename);
	else
	{
		// get size
		fseek(f, 0, SEEK_END);
		filesize = ftell(f);
		
		if ( (cdlsize * EXT_CDL_FIELDS) != filesize )
			errmsg=ExtCdlSize;
		else
		{
			fseek(f, 0, SEEK_SET);
			
			// allocate memory + read data
			cdl8000 = read_bin_block(f, cdlsize);
			cdlA000 = read_bin_block(f, cdlsize);
			cdlC000 = read_bin_block(f, cdlsize);
			cdlE000 = read_bin_block(f, cdlsize);
			cdlIndexLB = read_bin_block(f, cdlsize);
			cdlIndexUB = read_bin_block(f, cdlsize);

		}
	}
	
    if(f) fclose(f);
    
    return;
}


// parse header into struct
void parse_ines_header()
{
	header.prg	= romfile[4];
	header.chr	= romfile[5];
	header.map	= ((romfile[6] & 0xF0) >> 4) | (romfile[7] & 0xF0);
	header.mir	= romfile[6] & 0x09;
	header.sram	= romfile[6] & 0x02;
	
	physicalbankcount = (header.prg * 2);
}

//
bool is_code_or_data(long ptr)
{
	return ( (cdlfile[ptr] & (CODE | DATA)) != 0 );
}

long get_base_address(long ptr)
{
	return ((cdlfile[ptr] & BANK) << 11) | 0x8000;	
}


long get_bank_bits(long addr)
{
	switch (logicalbank[bank_id].size)
	{
		case 0x2000:
			return (addr & 0xE000);
		
		case 0x4000:
			return (addr & 0xC000);
			
		case 0x8000:
			return (addr & 0x8000);
			
		default:
			fatal_error("Invalid banksize! [%04X]", logicalbank[bank_id].size);
			return 0;
	}
}

bool is_same_bank(long addr1, long addr2)
{
	return ( get_bank_bits(addr1) == get_bank_bits(addr2) );
}

// get target address bank number
byte get_target_bank(long offset, long target_addr)
{
	int bank8k;
	switch (target_addr & 0xE000)
	{
		case 0x8000:	bank8k = cdl8000[offset];	break;
		case 0xA000:	bank8k = cdlA000[offset];	break;
		case 0xC000:	bank8k = cdlC000[offset];	break;
		case 0xE000:	bank8k = cdlE000[offset];	break;
		default:		return 0xFF;
	}

	if (bank8k < physicalbankcount)
		return physicalbankindex[bank8k];
	else
		fatal_error("target bank is out of bounds\n");
	
}

// read operand and convert to a string
void get_operand(long offset, char* str)
{
	long bank_mask = logicalbank[bank_id].size - 1;
	long target_addr, rel_val;
	byte target_bank;
	int i;
	
	switch (addr_mode)
	{
		case ABS:
		case ABSX:
		case ABSY:
			target_addr = romfile[offset] | (romfile[offset + 1] << 8);
			if (target_addr < 0x0100)									// force ABS instead of ZP
			{
				i = findlabel( 0xFF, target_addr );
				if (i < 0) fatal_error("!ZP label not found!");
				sprintf( str, "!%s", (*(labellist[i])).name );
			}
			else if (target_addr < 0x2000)								// internal ram (and mirrors)
			{
				i = findlabel( 0xFF, target_addr );
				if (i < 0) fatal_error("RAM label not found!");
				sprintf( str, "%s", (*(labellist[i])).name );
			}
			else if (target_addr < 0x6000)								// registers (or ???)
			{
				i = findlabel( 0xFF, target_addr );
				if (i < 0)
					sprintf( str, "$%04X", target_addr );
				else
					sprintf( str, "%s", (*(labellist[i])).name );
			}
			else if (target_addr < 0x8000)								// cartridge ram
			{
				i = findlabel( 0xFF, target_addr );
				if (i < 0) fatal_error("WRAM label not found!");
				sprintf( str, "%s", (*(labellist[i])).name);
			}
			else														// everything else
			{
				if ( (opcode == STA) || (opcode == STX) || (opcode == STY) )
					sprintf( str, "$%04X", target_addr );
				else
				{
					target_bank = get_target_bank(offset - 0x10, target_addr);
					//sprintf( str, "label_%02X_%04X", target_bank, target_addr);
					
					i = findlabel( target_bank, target_addr );
					if (i < 0) fatal_error("ABS label not found!");
					sprintf( str, "%s", labellist[i]->name);
				}
			}
			break;
			
		case IND:
			target_addr = romfile[offset] | (romfile[offset + 1] << 8);
			i = findlabel( 0xFF, target_addr );
			if (i < 0) fatal_error("Indirect label not found!");
			sprintf( str, "%s", (*(labellist[i])).name );
			break;

		case INDX:
		case INDY:
			target_addr = romfile[offset];
			i = findlabel( 0xFF, target_addr );
			if (i < 0) fatal_error("Indirect-ZP label not found!");
			sprintf( str, "%s", (*(labellist[i])).name );
			break;
		
		case ZP:
		case ZPX:
		case ZPY:
			target_addr = romfile[offset];
			i = findlabel( 0xFF, target_addr );
			if (i < 0) fatal_error("ZP label not found!");
			sprintf( str, "%s", (*(labellist[i])).name );
			break;
		
		case REL:
			rel_val = romfile[offset];
			if (rel_val > 0x80) rel_val = -((rel_val ^ 0xFF) + 1);
			target_addr = base_addr + ((offset - 0x10) & bank_mask) + 1 + rel_val;
			target_bank = get_target_bank(offset - 0x10, target_addr);
			i = findlabel( target_bank, target_addr );
			if (i < 0) fatal_error("Branch label not found!");
			sprintf( str, "%s", (*(labellist[i])).name );
			break;
			
		default:
			switch (opsize[addr_mode])
			{
				case 0:
					sprintf( str, "" );
					break;
					
				case 1:
					sprintf( str, "$%02X", romfile[offset] );
					break;
					
				case 2:
					sprintf( str, "$%04X", romfile[offset] | (romfile[offset + 1] << 8) );
					break;
			}
			break;

	}
}


// ================================================================================

// find all labels that need to be placed
void find_labels()
{
	char str[64];
	
	long sptr, bptr, vptr;
	byte rval, cval, target_bank;
	long target_addr, temp_addr, rel_val;
	
	for (bank_id = 0; bank_id < logicalbankcount; bank_id++)
	{
		bptr = logicalbank[bank_id].start;
		sptr = 0;
		
		// get vectors if in this bank
		if (logicalbank[bank_id].vectors)
		{
			vptr  = (0x10 + bptr + logicalbank[bank_id].size) - 6;

			sprintf( str, "NMI_%02X", bank_id );
			addlabel( bank_id, romfile[vptr + 0] | (romfile[vptr + 1] << 8), str );
			
			sprintf( str, "RESET_%02X", bank_id );
			addlabel( bank_id, romfile[vptr + 2] | (romfile[vptr + 3] << 8), str );
			
			sprintf( str, "IRQ_%02X", bank_id );
			addlabel( bank_id, romfile[vptr + 4] | (romfile[vptr + 5] << 8), str );	
		}

		while (sptr < logicalbank[bank_id].size)
		{
			//base_addr = get_base_address(bptr + sptr);

			if ( is_code_or_data(bptr + sptr) )
			{
				temp_addr = get_base_address(bptr + sptr);
			
				if (   (base_addr != temp_addr) && ( (base_addr + sptr) != (temp_addr + (sptr & 0x1FFF)) )  )
					base_addr = temp_addr;
			}
			
			rval = romfile[0x10 + bptr + sptr];
			cval = cdlfile[bptr + sptr];
			
			sptr++;
			
			if (cval & CODE)
			{
				opcode		= matrix_opc[rval];
				addr_mode	= matrix_adr[rval];
				
				switch (addr_mode)
				{
					case ABS:
					case ABSX:
					case ABSY:
						target_addr = romfile[0x10 + bptr + sptr] | (romfile[0x10 + bptr + sptr + 1] << 8);
						target_bank = get_target_bank(bptr + sptr, target_addr);
						
						// stores to $8000 - $FFFF are mapper registers (don't generate a label, these are printed with hardcoded addresses)
						if ( (target_addr < 0x8000) || ((opcode != STA) && (opcode != STX) && (opcode != STY)) )
							addlabel(target_bank, target_addr, 0);
							
						break;
					
					case IND:
					case INDX:
					case INDY:
						break;
					
					case ZP:
					case ZPX:
					case ZPY:
						target_addr = romfile[0x10 + bptr + sptr];
						target_bank = 0xFF;
						addlabel(target_bank, target_addr, 0);
						break;

					case REL:
						rel_val = romfile[0x10 + bptr + sptr];
						if (rel_val > 0x80) rel_val = -((rel_val ^ 0xFF) + 1);
						target_addr = base_addr + sptr + 1 + rel_val;
						target_bank = get_target_bank(bptr + sptr, target_addr);
						addlabel(target_bank, target_addr, 0);
						break;
				}

				sptr += opsize[addr_mode];
			}
		};
		
	}
}

// dissassemble a prg-bank to the outfile
void output_prg_bank(FILE *outfile, long &bptr, long& sptr)
{
	char str[512], operand[64];
	
	long bank_mask = logicalbank[bank_id].size - 1;
	byte rval, cval;
	int str_len, label_id;
	long temp_addr;
	
	// disassemble
	while (sptr < logicalbank[bank_id].size)
	{
		if ( is_code_or_data(bptr + sptr) )
		{
			temp_addr = get_base_address(bptr + sptr);

			if ( (base_addr + sptr) != (temp_addr + (sptr & 0x1FFF)) )
			{
				base_addr = temp_addr;
				if (base_addr != prev_base_addr)
				{
					sprintf( str, "\t.base ($ & $%04X) | $%04X\n", bank_mask, base_addr );
					fwrite((const void *)str, 1, strlen(str), outfile);
					prev_base_addr = base_addr;
				}
			}
		}

		rval = romfile[0x10 + bptr + sptr];
		cval = cdlfile[bptr + sptr];
		
		if (cval & SUB)
		{
			sprintf( str, "\n; ========================================\n");
			fwrite((const void *)str, 1, strlen(str), outfile);
		}		
		
		if ( (label_id = findlabelmirror(bank_id, sptr)) > -1 )
		{
			if ( !is_same_bank(base_addr, labellist[label_id]->address) )
			{
				base_addr = get_bank_bits(labellist[label_id]->address);
				sprintf( str, "\t.base ($ & $%04X) | $%04X\n", bank_mask, base_addr );
				fwrite((const void *)str, 1, strlen(str), outfile);
				prev_base_addr = base_addr;
			}

			sprintf( str, "%s:\n", labellist[label_id]->name );
			fwrite((const void *)str, 1, strlen(str), outfile);
		}
		

		if ( ((base_addr + sptr) == 0xFFFA) && (logicalbank[bank_id].vectors) )
		{
			sprintf( str, "\t.dw NMI_%02X\n", bank_id );
			sptr += 2;
		}
		else if ( ((base_addr + sptr) == 0xFFFC) && (logicalbank[bank_id].vectors) )
		{
			sprintf( str, "\t.dw RESET_%02X\n", bank_id );
			sptr += 2;
		}
		else if ( ((base_addr + sptr) == 0xFFFE) && (logicalbank[bank_id].vectors) )
		{
			sprintf( str, "\t.dw IRQ_%02X\n", bank_id );
			sptr += 2;
		}
		else
		{
			sptr++;
		
			if (cval & CODE)
			{
				opcode		= matrix_opc[rval];
				addr_mode	= matrix_adr[rval];
				
				if ( (opcode == BIT) & flag_expandbit)
				{
					if ( (label_id = findlabel(bank_id, base_addr + sptr)) > -1 )
					{
						sprintf( str, "\t.db $%02X\t\t; BIT\n", rval );
						fwrite((const void *)str, 1, strlen(str), outfile);
						continue;
					}
				}
	
				get_operand(0x10 + bptr + sptr, &operand[0]);
				sptr += opsize[addr_mode];
				sprintf( str, "\t%s %s%s%s\n", opmnem[opcode], ophead[addr_mode], operand, optail[addr_mode] );
				
			}
			else if (cval & DATA)
			{
				if (flag_datawidth < 2)
					sprintf(str, "\t.db $%02X\n", rval);
				else
				{
					str_len = sprintf( str, "\t.hex %02X", rval );
					for (int i = 1; (i < flag_datawidth) && (sptr < logicalbank[bank_id].size); i++)
					{
						if ( (label_id = findlabelmirror(bank_id, sptr)) > -1 )
							break;
						
						if ( ((base_addr + sptr) == 0xFFFA) && (logicalbank[bank_id].vectors) )
							break;
						
						rval = romfile[0x10 + bptr + sptr];
						cval = cdlfile[bptr + sptr];
						if (cval & DATA)
						{
							str_len += sprintf( str + str_len, "%02X", rval );
							sptr++;
						}
						else
							break;
					}
					str_len += sprintf( str + str_len, "\n" );
				}
			}
			else
			{
				unusedcount++;
				sprintf(str, "\t.db $%02X\n", rval);
			}
		}
			
		fwrite((const void *)str, 1, strlen(str), outfile);

	};
}






// ================================================================================

// dasm6n <rom> <cdl>
int main(int argc,char **argv) 
{
	int notoption;
	
	char str[512], filename[512];
	char *strptr;
	const char *name;
	FILE *outfile, *prgfile, *chrfile;

	long sptr, bptr, address, next_address, chr_size;
	int label_size;
	
	if(argc < 2)
	{
        showhelp();
        return EXIT_FAILURE;
    }

	// parse arguments
	notoption = 0;
    for(int i = 1; i < argc; i++)
	{
        if (*argv[i] == '-' || *argv[i] == '/')
		{
            switch(argv[i][1])
			{
                case 'h':
                case '?':
                    showhelp();
                    return EXIT_FAILURE;
                    
                case 'S':
                	flag_splitprg = 1;
                	break;
                    
                case 'E':
                    flag_expandbit = 1;
                    break;
                
                case 'R':
                	flag_regname = 1;
                	break;
                    
                case 'B':
                    flag_bankcfg = 1;
                    i++;
                    if (i < argc)
                    	bankfilename = argv[i];
                    else
                    	fatal_error("-B flag is missing <filename> argument");
                    break;
                    
                case 'W':
                    if(argv[i][2])
						flag_datawidth = atoi( &argv[i][2] );
                    break;
                
                case 'U':
                	flag_unused = 1;
                	break;
                	
                case 'D':
                	flag_debug = 1;
                	break;

                default:
                    fatal_error("unknown option: %s", argv[i]);
            }
        }
		else
		{
            if (notoption == 0)
                romfilename = argv[i];
            else if (notoption == 1)
                cdlfilename = argv[i];
            else if (notoption == 2)
                labelfilename = argv[i];
            else
                fatal_error("unused argument: %s", argv[i]);
            notoption++;
        }
    }
    
    
    // read rom file
    if (!romfilename)
        fatal_error("No rom file specified.");
    else
    {
    	fprintf( stdout, "reading rom\n" );
		romfile = read_bin(romfilename, romfilesize);
		if (errmsg) fputs(errmsg, stderr);
    }
    
    
	// read cdl files
	if (!cdlfilename)
		fatal_error("No cdl file specified.");
	else
	{
		fprintf( stdout, "reading cdl files\n" );
		// read legacy cdl file
		cdlfile = read_bin(cdlfilename, cdlfilesize);
		if (errmsg) fputs(errmsg, stderr);
		// read extended cdl file
		sprintf( filename, "%s.ext", cdlfilename );
		read_extended_cdl(filename, cdlfilesize);
		if (errmsg) fputs(errmsg, stderr);
	}

	
	// parse header
	parse_ines_header();
	set_bank_count();
	if (errmsg) fputs(errmsg, stderr);
	
	
	// read label list -AND- find labels
	initlabels();
	if (flag_regname)
	{
		add_registers_to_labels();	
	}
	if (labelfilename)
	{
		fprintf( stdout, "reading label list\n" );
		read_labels();
	}
	fprintf( stdout, "finding labels\n" );
	find_labels();
	qsort(labellist, labelcount, sizeof(label*), comparelabels);
	
	if (flag_debug)
	{
		outfile = fopen("debug.txt", "w");
		for (int i = 0; i < labelcount; i++)
		{
			sprintf(str, "%02X %04X %s\n", labellist[i]->bank, labellist[i]->address, labellist[i]->name);
			fwrite((const void *)str, 1, strlen(str), outfile);
		}
		fclose(outfile);
	}
	
	// get ouput filename
	strcpy(filename, romfilename);
	strptr = strrchr(filename, '.'); // strptr ='.'ptr
	if(strptr) if(strchr(strptr, '\\')) strptr = 0; // watch out for "dirname.ext\listfile"
	if(!strptr) strptr = filename + strlen(filename); // strptr -> inputfile extension
	strcpy(strptr, ".asm");
	
	// create file directory
	mkdir("file", 0755);
	
	// print constants
	if (flag_regname)
	{
		outfile = fopen("file\\const.asm", "w");
		fprintf( stdout, "generating const definitions\n" );
		for (int i = 0; i < ppu_reg_count; i++)
		{
			size_t l = strlen(ppu_reg[i]);
			if (l < 8)
				sprintf( str, "%s\t\tequ $%04X\n", ppu_reg[i], ppu_reg_addr[i] );
			else
				sprintf( str, "%s\tequ $%04X\n", ppu_reg[i], ppu_reg_addr[i] );
			fwrite((const void *)str, 1, strlen(str), outfile);
		}
		
		for (int i = 0; i < apu_reg_count; i++)
		{
			size_t l = strlen(apu_reg[i]);
			if (l < 8)
				sprintf( str, "%s\t\tequ $%04X\n", apu_reg[i], apu_reg_addr[i] );
			else
				sprintf( str, "%s\tequ $%04X\n", apu_reg[i], apu_reg_addr[i] );
			fwrite((const void *)str, 1, strlen(str), outfile);
		}
	}

	// print ram
	outfile = fopen("file\\ram.asm", "w");

		fprintf( stdout, "generating ram definitions\n" );
		sprintf( str, "\n.ram $0000\n" );
		fwrite((const void *)str, 1, strlen(str), outfile);
		for (int i = 0; i < labelcount; i++)
		{
			if ((*(labellist[i])).bank == 0xFF)
			{
				address = (*(labellist[i])).address;
				name = (*(labellist[i])).name;
				
				if ( (i + 1) < labelcount )
				{
					next_address = (*(labellist[i+1])).address;
					label_size = next_address - address;
				}
				else
				{
					if (address >= 0x6000)
						label_size = 0x8000 - address;
					else
						label_size = 1;
				}
				
				if (address < 0x0100)
					sprintf( str, "%s:\t.dsb %d\n", name, label_size );
				else if (address < 0x2000)
				{
					if (next_address >= 0x2000)
						sprintf( str, "%s:\t.dsb %d\n.endram\n\n.wram $6000\n", name, 0x0800 - (address & 0x07FF) );
					else
						sprintf( str, "%s:\t.dsb %d\n", name, label_size );
				}
				else if ( (address >= 0x6000) && (address < 0x8000) )
					sprintf( str, "%s:\t.dsb %d\n", name, label_size );
				else
					continue;
					
				fwrite((const void *)str, 1, strlen(str), outfile);
			}
		}
		sprintf( str, ".endwram\n" );
		fwrite((const void *)str, 1, strlen(str), outfile);
		
	fclose(outfile);

	outfile = fopen(filename, "w");
	
		// print ppu/apu registers if used
		if (flag_regname)
		{
			sprintf( str, "\n\t.include \"file\\const.asm\"\n" );
			fwrite((const void *)str, 1, strlen(str), outfile);
		}
	
		// print ram include
		sprintf( str, "\n\t.include \"file\\ram.asm\"\n\n" );
		fwrite((const void *)str, 1, strlen(str), outfile);



		// print ines header
		fprintf( stdout, "generating ines header\n" );
		sprintf( str, "\n\t.db \"NES\", $1A\n\t.db $%02X\n\t.db $%02X\n\t.db $%02X\n\t.db $%02X\n\t.dsb 8, $00\n",
			header.prg, header.chr, ((header.map & 0x0F) << 4) | header.mir | header.sram, (header.map & 0xF0) );
		fwrite((const void *)str, 1, strlen(str), outfile);



		// print each bank
		fprintf( stdout, "disassembling prg-rom\n" );
		for (bank_id = 0; bank_id < logicalbankcount; bank_id++)
		{
			fprintf( stdout, " bank %d..", bank_id );
			
			unusedcount = 0;
			
			bptr = logicalbank[bank_id].start;
			sptr = 0;

			// print bank number
			sprintf( str, "\n\n\t.bank $%02X\n", bank_id );
			fwrite((const void *)str, 1, strlen(str), outfile);
				
			base_addr = get_base_address(bptr + sptr);
				
			sprintf( str, "\t.base $%04X\n", base_addr );
			fwrite((const void *)str, 1, strlen(str), outfile);
			
			prev_base_addr = base_addr;
			
			if (flag_splitprg)
			{
				sprintf( filename, "file\\prg%02X.asm", bank_id );
				prgfile = fopen(filename, "w");
				output_prg_bank(prgfile, bptr, sptr);
				fclose(prgfile);
				
				sprintf( str, "\t.include \"file\\prg%02X.asm\"\n", bank_id );
				fwrite((const void *)str, 1, strlen(str), outfile);
			}
			else
			{
				output_prg_bank(outfile, bptr, sptr);
			}
			
			if (flag_unused)
			{
				double f = (static_cast<double>(unusedcount) / static_cast<double>(logicalbank[bank_id].size)) * 100.0;
				fprintf( stdout, "\t%4X (%5.2f%%)", unusedcount, f );
			}

			fprintf( stdout, "\n");
			
		}
		
		// write chr-rom (if present)
		if (header.chr > 0)
		{
			fprintf( stdout, "writing chr-rom\n" );
			sprintf(str, "\n\n\n\t.incbin \"file\\gfx.chr\"\n");
			fwrite((const void *)str, 1, strlen(str), outfile);
			
			bptr = header.prg * 0x4000;
			chr_size = header.chr * 0x2000;
			
			chrfile = fopen("file\\gfx.chr","wb");
			fwrite( &(romfile[0x10 + bptr]), sizeof(byte), chr_size, chrfile );
			fclose(chrfile);
		}
	
	fclose(outfile);
	
	free(labellist);
	
	delete[] cdlfile;
	delete[] cdl8000;
	delete[] cdlA000;
	delete[] cdlC000;
	delete[] cdlE000;
	delete[] cdlIndexLB;
	delete[] cdlIndexUB;
	
	delete[] romfile;
	
	delete[] logicalbank;
	delete[] physicalbankindex;
	
	return 0;

}
