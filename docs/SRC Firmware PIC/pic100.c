/* MD-100 compatible solid state mass storage device,
   to be compiled with the Microchip MCC18 compiler,
   microprocessor PIC18F4520 */

#include <p18cxxx.h>
#include <string.h>	/* memcmp, memcpy, memset */
#include <i2c.h>
#include "pic100.h"

#pragma config OSC = INTIO67
#pragma config PWRT = ON
#pragma config BOREN = SBORDIS
#pragma config BORV = 1
#pragma config WDT = OFF
#pragma config MCLRE = ON
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config XINST = OFF
#pragma config DEBUG = OFF

#define	FOSC	4000000		/* clock frequency */

#define	SECTORS_PER_TRACK	16

/* file system parameters */
#define	SECTORS_FAT	4
#define	SECTORS_DIR	12
#define	START_FAT	0
#define	START_DIR	4
#define	START_DATA	16
#define	MAX_FILES	16	/* number of handles */
#define	SIZE_FILE_NAME	(8+3)
#define	SIZE_BLOCK	4	/* 4 sectors per block */
#define	SIZE_RECORD	256
#define	SIZE_FAT_ENTRY	2
#define	SIZE_DIR_ENTRY	sizeof (DIR_ENTRY)
#define	DIR_ENTRIES	(SECTORS_DIR * SIZE_RECORD / SIZE_DIR_ENTRY)
#define	RESERVED	(START_DATA / SIZE_FAT_ENTRY)
#define	EOF_CHAR	0x1A

#define	PAGESIZE	0x80	/* EEPROM Microchip 24LC1025 */
#define	SIZE_FAT_CHUNK	16	/* number of FAT entries read from the EEPROM
 while searching for a free cluster, suboptimal number would slow down the
 search (EEPROM access overhead when too low, often excessive amount of data
 read from the EEPROM when too high) */

/* port pins */
#define	SDA	RC4	/* I2C bus */
#define	SCL	RC3	/* I2C bus */
#define	GRN_LED	RC2	/* green LED driven in the PWM mode (CCP1) */
#define	PGMDATA	RB7	/* programming mode data input/output */
#define	PGMCLK	RB6	/* programming mode clock input */
#define	FD_DIR	RB0	/* transfer direction strobe signal input (P2) */
#define	FD_ACK	RA5	/* transfer direction acknowledge signal output (P0) */
#define	FD_PWR	RA4	/* power control signal input (P3) */


#pragma udata udata1

byte media;		/* set bits mark populated 64KB EEPROM blocks */
byte first_cntrl;	/* control byte for the first EEPROM block */
int dir_index;		/* index of a directory entry */
DIR_ENTRY debuf1;	/* directory entry read from / written to EEPROM */
DIR_ENTRY debuf2;	/* directory entry buffer ($2280) */
byte access[MAX_FILES];	/* access mode for each of 16 files ($00A9) */
word position[MAX_FILES];	/* current cluster index for each file on bits
 8..0 ($0110) and which sector in those clusters on bits 13..12 ($0130) */
volatile byte pspdata;	/* data received from the PSP port */

/* timers decremented every 1ms to 0 */
volatile byte prog_timer;	/* used for EEPROM programming */
volatile byte idle_timer;	/* sleep mode after time-out */

union {
  byte all;
  struct {
    unsigned END_OF_FILE:1;	/* ... or no more files */
    unsigned FILE_READ_ONLY:1;
    unsigned DATA_ERROR:1;
    unsigned RENAME_FAILED:1;
    unsigned FILE_FOUND:1;
    unsigned ALREADY_OPENED:1;	/* file already opened or file handle already
				   in use */
    unsigned ACCESS_ERROR:1;
    unsigned FILE_NOT_OPENED:1;
  } bits;
} fd_status;	/* status code of the operation ($0054) */

union {
  byte all;
  struct {
    unsigned LOW_BATTERIES:1;
    unsigned WRITE_PROTECTED:1;	/* disk write protected */
    unsigned NOT_READY:1;	/* disk not inserted */
    unsigned EEPROM_ACCESS:1;
    unsigned XX4:1;
    unsigned INVALID_COMMAND:1;
    unsigned DISK_FULL:1;
    unsigned OPENED_FILES:1;	/* format failed because of opened files */
  } bits;
} fd_error;	/* error code of the operation ($0055) */

word fd_counter;	/* counter of transferred bytes ($0062, $0063) */
byte fd_command;	/* command code ($0073) */
byte fd_handle;		/* file handle ($007A) */
word max_cluster;	/* ($0093, $0094) */

#pragma udata udata2

DIR_ENTRY fcb_table[MAX_FILES];	/* table of 16 File Control Blocks ($2290) */

#pragma udata udata3

byte secbuf[SIZE_RECORD];	/* sector buffer ($2400) */


#pragma code high_vector = 0x08
void high_vector (void)
{
  _asm
    goto high_isr
  _endasm
}
# pragma code


# pragma interrupt high_isr
void high_isr (void)
{
  if (PIR1bits.PSPIF)
  {
    PIR1bits.PSPIF = 0;
    if (TRISEbits.IBF)
    {
      pspdata = PORTD;
    }
    idle_timer = 0xFF;
    TRISCbits.GRN_LED = 0;	/* green LED on */
  }
  if (INTCONbits.INT0IF)
  {
    INTCONbits.INT0IF = 0;
    idle_timer = 0xFF;
    TRISCbits.GRN_LED = 0;	/* green LED on */
  }
  if (PIR1bits.TMR2IF)
  {
    PIR1bits.TMR2IF = 0;
    if (prog_timer)
    {
      --prog_timer;
    }
    if (idle_timer && !--idle_timer)
    {
      TRISCbits.GRN_LED = 1;	/* green LED off */
    }
  }
}
#pragma code


/* macro definitions */

/* receive a byte from the host, decrement 'fd_counter', terminate the command
   by sending 'fd_error.all' to the host when overflow */
#define	GetByteAndCnt	if (transf_and_cnt (0)) { put_byte (fd_error.all); return; }


/* receive a file name from the host, terminate the command by sending
   'fd_error.all' to the host when 'fd_counter' caused an overflow */
#define	GetFileName	if (get_block ((byte*) &debuf2, SIZE_DIR_ENTRY)) { put_byte (fd_error.all); return; }


/* if the request from the host is not equal zero, then terminate the command
   by sending 'fd_error.all' to the host */
#define	CheckRequest	if (pspdata) { put_byte (fd_error.all); return; }


/* if the 'fd_error.all' value is not equal zero, then terminate the command
   by sending 'fd_error.all' to the host */
#define	CheckFdError	if (fd_error.all) { put_byte (fd_error.all); return; }


byte ee_presence (byte control /* R/W bit should be 0 */)
{
  byte x;

  IdleI2C ();
  StartI2C ();
  while (SSPCON2bits.SEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  SSPBUF = control;
  if (SSPCON1bits.WCOL)
  {
    StopI2C ();
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  x = SSPCON2bits.ACKSTAT;
  StopI2C ();
  while (SSPCON2bits.PEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  return (x) ? -2 /* Not Ack */ : 0 /* Ack */;
}


/* scan the I2C bus for populated EEPROM blocks */
void ee_scan (void)
{
  byte control;

  media = 0;
  max_cluster = 0;
  for (control = 0xA0; control < 0xA0+16; control += 2)
  {
    media >>= 1;
    if (!ee_presence (control))
    {
      media |= 0x80;
      max_cluster += 64;
    }
  }
}


/* This is an equivalent of the original EESequentialRead routine from the
   I2C library, but with two bytes of address data. */
byte ee_read (byte control, word address, byte *rdptr, byte length)
{
  idle_timer = 0xFF;
  IdleI2C ();
  StartI2C ();
  while (SSPCON2bits.SEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  SSPBUF = control;
  if (SSPCON1bits.WCOL)
  {
    StopI2C ();  
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  if (SSPCON2bits.ACKSTAT)
  {
    StopI2C ();
    return -2;			/* Not Ack error */
  }

  SSPBUF = address>>8;
  if (SSPCON1bits.WCOL)
  {
    StopI2C ();
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  if (SSPCON2bits.ACKSTAT)
  {
    StopI2C ();
    return -2;			/* Not Ack error */
  }

  SSPBUF = address;
  if (SSPCON1bits.WCOL)
  {
    StopI2C ();
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  if (SSPCON2bits.ACKSTAT)
  {
    StopI2C ();
    return -2;			/* Not Ack error */
  }

  RestartI2C ();
  while (SSPCON2bits.RSEN);

  SSPBUF = control+1;
  if (SSPCON1bits.WCOL)
  {
    StopI2C ();
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  if (SSPCON2bits.ACKSTAT)
  {
    StopI2C ();
    return -2;			/* Not Ack error */
  }

  do {
    SSPCON2bits.RCEN = 1;
    while (!SSPSTATbits.BF);
    *rdptr++ = SSPBUF;
    while (SSPCON2bits.RCEN);   
    if (PIR2bits.BCLIF)
    {
      return -1;		/* bus collision */ 
    }
    SSPCON2bits.ACKDT = (--length) ? 0 : 1 /* no Ack when last byte received */;
    SSPCON2bits.ACKEN = 1;
    while (SSPCON2bits.ACKEN);
  } while (length);

  StopI2C ();
  while (SSPCON2bits.PEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  return 0;			/* no error */
}


/* This is a modification of the original EEPageWrite routine from the I2C
   library. */
byte ee_write (byte control, word address, byte *wrptr, byte length)
{
  idle_timer = 0xFF;
  IdleI2C ();
  StartI2C ();
  while (SSPCON2bits.SEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  SSPBUF = control;
  if (SSPCON1bits.WCOL)
  {
    StopI2C (); 
    return -3;			/* write collision */
  }
  while (SSPSTATbits.BF);
  IdleI2C ();

  do {

/* write a memory block to the EEPROM, the start bit and the control byte are
  already successfully transmitted at this point */

    if (SSPCON2bits.ACKSTAT)
    {
      StopI2C (); 
      return -2;		/* Not Ack error */
    }

    SSPBUF = address>>8;
    if (SSPCON1bits.WCOL)
    {
      StopI2C ();
      return -3;		/* write collision */
    }
    while (SSPSTATbits.BF);
    IdleI2C ();

    if (SSPCON2bits.ACKSTAT)
    {
      StopI2C ();
      return -2;		/* Not Ack error */
    }

    SSPBUF = address;
    if (SSPCON1bits.WCOL)
    {
      StopI2C ();
      return -3;		/* write collision */
    }
    while (SSPSTATbits.BF);
    IdleI2C ();

    if (SSPCON2bits.ACKSTAT)
    {
      StopI2C ();
      return -2;		/* Not Ack error */
    }

    do {
      SSPBUF = *wrptr++;
      if (SSPCON1bits.WCOL)
      {
        StopI2C ();
        return -3;		/* write collision */
      }
      while (SSPSTATbits.BF);
      IdleI2C ();

      if (SSPCON2bits.ACKSTAT)
      {
        StopI2C ();
        return -2;		/* Not Ack error */
      }
    } while (--length && (++address % PAGESIZE));

    StopI2C ();
    while (SSPCON2bits.PEN);
    if (PIR2bits.BCLIF)
    {
      return -1;		/* bus collision */
    }

/* wait until the write cycle to the EEPROM is complete */

    IdleI2C ();
    StartI2C ();
    while (SSPCON2bits.SEN);
    if (PIR2bits.BCLIF)
    {
      return -1;		/* bus collision */
    }

    SSPBUF = control;
    if (SSPCON1bits.WCOL)
    {
      StopI2C ();
      return -3;		/* write collision */
    }
    while (SSPSTATbits.BF);
    IdleI2C ();

    prog_timer = 20;
    while (SSPCON2bits.ACKSTAT && prog_timer)
    {
      RestartI2C ();
      while (SSPCON2bits.RSEN);
      if (PIR2bits.BCLIF)
      {
        return -1;		/* bus collision */
      }

      SSPBUF = control;
      if (SSPCON1bits.WCOL)
      {
        StopI2C ();
        return -3;		/* write collision */
      }
      while (SSPSTATbits.BF);
      IdleI2C ();
    }

  } while (length && prog_timer);

  StopI2C ();
  while (SSPCON2bits.PEN);
  if (PIR2bits.BCLIF)
  {
    return -1;			/* bus collision */
  }

  return (prog_timer) ? 0 /* no error */ : -4 /* operation timed out */;
}


/* calculate the EEPROM control byte for the sector 'x' */
byte ee_cntrl (word x)
{
  byte block, mask, control;

  block = x >> 8;
  mask = 0x01;
  for (control = 0xA0; control < 0xA0+16; control += 2)
  {
    if (media & mask)
    {
      if (!block)
      {
        return control;
      }
      --block;
    }
    mask <<= 1;
  }
/* sector out of EEPROM space */
  return 0;
}


/* read sector 'x' to the 'secbuf' */
void read_sector (word x)
{
  byte y;

  if (	!(y = ee_cntrl(x)) ||
	ee_read (y, x * SIZE_RECORD, secbuf, (byte) SIZE_RECORD))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


/* write the 'secbuf' to the sector 'x' */
void write_sector (word x)
{
  byte y;

  if (	!(y = ee_cntrl(x)) ||
	ee_write (y, x * SIZE_RECORD, secbuf, (byte) SIZE_RECORD))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


void read_sec1 (word cluster, byte sector)
{
  read_sector ((cluster & 0x01FF) * SIZE_BLOCK + (word) sector);
}


void write_sec1 (word cluster, byte sector)
{
  write_sector ((cluster & 0x01FF) * SIZE_BLOCK + (word) sector);
}


/* read a directory entry 'x' to the 'debuf1' */
void read_dir_entry (word x)
{
  if (ee_read (first_cntrl, x * SIZE_DIR_ENTRY + START_DIR * SIZE_RECORD,
	(byte*) &debuf1, SIZE_DIR_ENTRY))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


/* write the 'debuf1' to the directory entry 'x' */
void write_dir_entry (word x)
{
  if (ee_write (first_cntrl, x * SIZE_DIR_ENTRY + START_DIR * SIZE_RECORD,
	(byte*) &debuf1, SIZE_DIR_ENTRY))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


/* value of a word in big endian order pointed to by 'ptr' */
word big_endian_word (byte *ptr)
{
  byte x;

  x = *ptr;
  return (((word) x) << 8) | (word) *++ptr;
}


/* read a FAT entry 'x' */
word read_fat_entry (word x)
{
  byte buf[SIZE_FAT_ENTRY];

  x &= 0x01FF;
  if (ee_read (first_cntrl, x * SIZE_FAT_ENTRY + START_FAT * SIZE_RECORD,
	buf, SIZE_FAT_ENTRY))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
  return big_endian_word (buf);
}


/* read 'i' FAT entries starting from 'x' to the 'secbuf'  */
void read_fat_entries (word x, byte i)
{
  x &= 0x01FF;
  if (ee_read (first_cntrl, x * SIZE_FAT_ENTRY + START_FAT * SIZE_RECORD,
	secbuf, i * SIZE_FAT_ENTRY))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


/* write 'y' to a FAT entry 'x' */
void write_fat_entry (word x, word y)
{
  byte buf[SIZE_FAT_ENTRY];

  buf[0] = y >> 8;
  buf[1] = y;
  x &= 0x01FF;
  if (ee_write (first_cntrl, x * SIZE_FAT_ENTRY + START_FAT * SIZE_RECORD,
	buf, SIZE_FAT_ENTRY))
  {
    fd_error.bits.EEPROM_ACCESS = 1;
  }
}


void write_last_fat_entry (word cluster, byte sector)
{
  word x;

  x = (cluster & 0x01FF) | 0xC000 /* used entry + end of chain */;
  if (sector & 0x01)
  {
    x |= 0x1000;
  }
  if (sector & 0x02)
  {
    x |= 0x2000;
  }
  write_fat_entry (cluster, x);
}


/* receive the 16-bit number of transferred bytes 'fd_counter' from the host */
void get_num_bytes (void)
{
  transfer (0);
  fd_counter = (word) pspdata;		/* number of bytes, LSB */
  transfer (0);
  fd_counter |= ((word) pspdata) << 8;	/* number of bytes, MSB */
}


/* receive 'size' bytes from the host to a memory area pointed to by 'ptr',
   return a value <> 0 when 'fd_counter' overflow occurred */
byte get_block (byte *ptr, word size)
{
  do {
    *ptr++ = pspdata;
    if (!--size)
    {
      return 0;
    }
  } while (!transf_and_cnt (0));
  return 1;
}


/* send 'size' bytes pointed to by 'ptr' to the host */
byte put_block (byte *ptr, word size)
{
  while (size)
  {
    wait_for_rd_mode ();
    if (pspdata)
    {
      put_byte (fd_error.all);
      return 1;		/* transfer terminated by the host */
    }
    put_byte (*ptr++);
    --size;
  }
  return 0;		/* OK */
}


/* receive 'fd_counter'+1 bytes from the host to the 'secbuf'
   (replaces duplicate code blocks: $077B..$0793, $12BA..$12D3+$12DB..$12DE,
   $144D..$1468) */
byte* get_record (void)
{
  byte *ptr;

  ptr = secbuf;
  while (1)
  {
    *ptr = pspdata;
    if (ptr < secbuf+SIZE_RECORD-1)
    {
      ++ptr;
    }
    if (fd_counter == 0)
    {
      break;
    }
    --fd_counter;
    transfer (0);
  }
  return ptr;
}


/* find the last cluster of the file starting from a given index
   (replaces duplicate code blocks: $093E..$095A, $13AA..$13CB) */
word find_last_clus (word x /* cluster index */)
{
  do {
    x = read_fat_entry (x);	/* last chain entry points to itself */
  } while (!(x & 0x4000) && !fd_error.all);
  return x;
}


/* free the FAT chain starting from a given index up to the end
   (replaces duplicate code blocks: $0A26..$0A4D, $0C50..$0C77) */
void free_fat_chain (word x /* cluster index */)
{
  word y;

  do {
    y = read_fat_entry (x);
    write_fat_entry (x, 0);
    x = y;
  } while (!(y & 0x4000) && !fd_error.all);
}


/* wait until the host sends a byte then switches back to read mode */
void wait_for_rd_mode (void)
{
  while (PORTBbits.FD_DIR)
  {
    if (TRISCbits.GRN_LED)
    {
      _asm
      sleep
      _endasm
    }
  }
}


void main (void)
{
  OSCCON = 0x60;		/* 4MHz, only PIC18F4520 */
/* unused pins are configured as outputs */
  ADCON1 = 0x0F;		/* digital I/O, value 0x07 for PIC18F452 */
  TRISA = 0x10;
  TRISB = 0xE1;
  TRISC = 0xDC;
  TRISD = 0xFF;
  TRISE = 0x17;			/* enable the PSP port */
  PORTD = 0x55;			/* identification byte (Option Code) */
  PORTAbits.FD_ACK = 0;
  idle_timer = 0;

/* the CCP1 pin drives a green LED in the PWM mode
   Timer2 generates an interrupt every 1ms
   external interrupt0 on falling edge */
#define	val1 (FOSC/4/4/1000-1)
#define val2 ((val1*4)/5)	/* PWM duty cycle */
  PR2 = val1;
  CCPR1L = val2;
  T2CON = 0x1C;		/* Timer2 enabled, postscaler by 4 */
  CCP1CON = 0x0C;	/* PWM mode */
  RCONbits.IPEN = 1;	/* enable priority levels */
  IPR1 = 0x82;		/* PSP+Timer2 high priority */
  PIE1 = 0x82;		/* PSP+Timer2 interrupt enable */
  INTCON2bits.INTEDG0 = 0;	/* external interrupt0 on falling edge */
  INTCONbits.INT0IE = 1;	/* enable external interrupt0 */
  INTCONbits.GIEH = 1;	/* enable high priority peripheral interrupts */

/* I2C */
  SSPADD = FOSC/4/100000-1;	/* SCL frequency = ca. 100kHz */
  OpenI2C (MASTER, SLEW_OFF);

  ee_scan ();
/* if (!media) then ??? */
  first_cntrl = ee_cntrl (0);
  dir_index = 0;
  close_all ();

  while (1)
  {
/* 01D2: */
    fd_error.all = 0;
    fd_status.all = 0;
    wait_for_rd_mode ();
    fd_command = pspdata;
    switch (fd_command & 0xF0)
    {
      case 0x00:	/* read a directory entry */
        fd_cmd_0x ();
        break;
      case 0x10:	/* write to a file */
        fd_cmd_1x ();
        break;
      case 0x20:	/* read from a file */
        fd_cmd_2x ();
        break;
      case 0x30:	/* open file */
        fd_cmd_3x ();
        break;
      case 0x40:	/* close file(s) */
        fd_cmd_40 ();
        break;
      case 0x50:	/* delete file */
        fd_cmd_50 ();
        break;
      case 0x60:	/* rename file */
        fd_cmd_60 ();
        break;
      case 0x70:	/* write physical sector */
        fd_cmd_70 ();
        break;
      case 0x80:	/* read physical sector */
        fd_cmd_80 ();
        break;
      case 0x90:	/* format disk */
        fd_cmd_90 ();
        break;
      case 0xc0:	/* get the file size */
        fd_cmd_c0 ();
        break;
      case 0xd0:	/* get the free EEPROM space */
        fd_cmd_d0 ();
        break;
      default:
        fd_error.bits.INVALID_COMMAND = 1;
        put_byte (fd_error.all);
    }
  }
}


/* $02AB: command $90 - format disk */
void fd_cmd_90 (void)
{
/* $02E7: check if all files are closed */
  for (fd_handle=0; fd_handle<MAX_FILES; ++fd_handle)
  {
    if (fcb_table[fd_handle].type)
    {
      fd_error.bits.OPENED_FILES = 1;
      break;
    }
  }
  memset ((void*) secbuf, 0xFF, RESERVED);
  memset ((void*) secbuf+RESERVED, 0, SIZE_RECORD-RESERVED);
  for (fd_counter=START_FAT; !fd_error.all && fd_counter<START_DATA;
	++fd_counter)
  {
    write_sector (fd_counter);
    memset ((void*) secbuf, 0, RESERVED);
  }
  put_byte (fd_error.all);
}


/* $02D8: transmit data byte to the host */
void put_byte (byte x)
{
  TRISEbits.OBF = 0;
  PORTD = x;
  PORTAbits.FD_ACK = 1;		/* data ready for the host */
/* wait until the host reads the byte and switches to write mode */
  while (!PORTBbits.FD_DIR);
  PORTAbits.FD_ACK = 0;		/* acknowledge the host write mode */
}


/* $0705: single transfer transaction - send a byte to the host, then wait
   until the host sends a byte */
void transfer (byte x)
{
  put_byte (x);
  wait_for_rd_mode ();
}


/* $0711: command $80 - read physical sector */
void fd_cmd_80 (void)
{
  word i;

  transfer (0);				/* track */
  i = SECTORS_PER_TRACK * (word) pspdata;
  transfer (0);				/* sector */
  i += (word) --pspdata;	/* disk sectors are numbered from 1 up */
  read_sector (i);
  put_byte (fd_error.all);
  if (!fd_error.all)
  {
    put_block (secbuf, SIZE_RECORD);
  }
}


/* $074B: command $70 - write physical sector */
void fd_cmd_70 (void)
{
  word i, y;

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt				/* track */
  i = SECTORS_PER_TRACK * (word) pspdata;
  GetByteAndCnt				/* sector */
  i += (word) --pspdata;	/* disk sectors are numbered from 1 up */
  if (!transf_and_cnt (0))
  {
    y = fd_counter;
    get_record ();
    if (y == SIZE_RECORD-1)
    {
      write_sector (i);
    }
  }
  put_byte (fd_error.all);
}


/* $07A5: single transfer transaction,
   decrement the 16-bit 'fd_counter',
   return a value <> 0 when overflow */
byte transf_and_cnt (byte x)
{
  put_byte (x);
  if (fd_counter == 0)
  {
    return 1;
  }
  --fd_counter;
  wait_for_rd_mode ();
  return 0;
}


/* $0801: command $3x - open a file */
void fd_cmd_3x (void)
{
  DIR_ENTRY *myfcb;
  byte n, *ptr;
  word x;
  int i;

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt
  fd_handle = pspdata;
  GetByteAndCnt
  GetFileName
/* check whether the file isn't already opened or the handle already taken */
  for (n=0; n<MAX_FILES; ++n)
  {
    myfcb = &fcb_table[n];
    if (myfcb->type && (n==fd_handle ||
	!memcmp ((void*) &debuf2.name, (void*) &myfcb->name, SIZE_FILE_NAME)))
    {
      fd_status.bits.ALREADY_OPENED = 1;
      finish1 ();
      return;
    }
  }
/* $0896: copy the specified file name to the 'fcb_table' */
  myfcb = &fcb_table[fd_handle];
  memcpy ((void*) myfcb, (void*) &debuf2, SIZE_DIR_ENTRY);
/* $08C8: store the file access mode */
  access[fd_handle] = fd_command;
/* $08DB: */
  dir_index = -1;	/* no free directory entry found yet */
  i = find_dir_entry (-1);

  if (i >= 0)
  {
/* $08E9: open an existing file */
    myfcb->ublk = debuf1.ublk;
    myfcb->lblk = debuf1.lblk;
    x = big_endian_word (&debuf1.ublk);
/* $092D: */
    if (debuf1.attr)
    {
      fd_status.bits.FILE_READ_ONLY = 1;
    }

    if (fd_command & 0x04)
    {
/* $093B: open an existing file for append */
      if (!fd_status.bits.FILE_READ_ONLY)
      {
        x = find_last_clus (x);
        if (!fd_error.all)
        {
/* $095B: */
          read_sec1 (x, last_sectors (x));
          if (!fd_error.all)
          {
/* $0975: */
            ptr = end_of_record ();
            finish3 ((ptr) ? ptr-secbuf : 0);
            return;
          }
        }
      }
    }

    else if (fd_command & 0x02)
    {
/* $0A05: open an existing file for input */
      position[fd_handle] = x & 0x01FF;
      finish3 (0);		/* no terminating record will be returned */
      return;
    }

    else if (fd_command & 0x01)
    {
/* $0A20: open an existing file for random output */
      finish3 (0);		/* no terminating record will be returned */
      return;
    }

    else
    {
/* $0A23: open an existing file for sequential output */
      if (!fd_status.bits.FILE_READ_ONLY)
      {
/* $0A26: */
        free_fat_chain (x);
        CheckFdError
/* $0A4E: */
        debuf1.type = debuf2.type;
        dir_index = i;
/* $0A72: */
        create_file ();
        if (!(fd_error.all))
        {
          myfcb->ublk = debuf1.ublk;
          myfcb->lblk = debuf1.lblk;
          finish1 ();
          return;
        }
      }
    }
  }

  else if (i == -1)
  {
/* $0A69: no existing file found, create a new one */
    if (!(fd_command & 0x06))		/* no input or append allowed */
    {
/* $0A78: */
      if (dir_index < 0)
      {
/* no free directory entry available */
        fd_error.bits.DISK_FULL = 1;
      }
      else
      {
/* $0A72: */
        memcpy ((void*) &debuf1, (void*) &debuf2, SIZE_DIR_ENTRY);
        create_file ();
        if (!(fd_error.all))
        {
          myfcb->ublk = debuf1.ublk;
          myfcb->lblk = debuf1.lblk;
          finish1 ();
          return;
        }
      }
    }
  }

/* else an EEPROM access error occurred */

/* $0A7A: failed to open the file */
  myfcb->type = 0;		/* clear the 'fcb_table' entry */
  CheckFdError
  finish1 ();
}


/* $0988: common closing sequence */
void finish3 (word x /* size of the terminating record, 0 if not append */)
{
  x += 0x11;			/* status byte + directory entry size */
  transfer (0);
/* $0996: transmit the number of returned bytes to the host */
  CheckRequest
  transfer ((byte) x);		/* LSB */
  CheckRequest
  transfer ((byte) (x>>8));	/* MSB */
/* $09B8: transfer the 'fd_status' followed by the directory entry */
  CheckRequest
  fd_status.bits.FILE_FOUND = 1;
  put_byte (fd_status.all);
  if (!put_block ((byte*) &debuf1, SIZE_DIR_ENTRY))
  {
/* $09E2: transfer the contents of the terminating record */
    x -= 0x11;			/* back to the size of terminating record */
    put_block (secbuf, x);
  }
}


/* $0A72: create new file, sets 'fd_error.bits' if failed,
   expects directory entry data in 'debuf1' and an index of the directory
   entry in 'dir_index' */
void create_file (void)
{
  word x;

  x = find_free_clus (START_DATA/SIZE_BLOCK);
  if (!fd_error.all)
  {
/* $0AA6: save the number of the starting cluster to the directory entry */
    debuf1.ublk = x >> 8;
    debuf1.lblk = x;
/* $0AD6: save the directory entry to the EEPROM */
    write_dir_entry ((word) dir_index);
    if (!fd_error.all)
    {
/* $0ADD: allocate the data cluster in the FAT */
      write_last_fat_entry (x, 0);
      if (!fd_error.all)
      {
        clear_secbuf ();
        write_sec1 (x, 0);
      }
    }
  }
}


/* $0AFE: erase the contents of the 'secbuf' */
void clear_secbuf (void)
{
  secbuf[0] = EOF_CHAR;
  memset ((void*) secbuf+1, 0, SIZE_RECORD-1);
}


/* $0B2C: common closing sequence */
void finish1 (void)
{
  transfer (0);
  CheckRequest
  transfer (0x01);		/* number of returned bytes, LSB */
  CheckRequest
  transfer (0x00);		/* number of returned bytes, MSB */
  CheckRequest
  put_byte (fd_status.all);
}


/* $0BDE: command $50 - delete file */
void fd_cmd_50 (void)
{
  int i;	/* index of the directory entry */
  word x;	/* index of the starting cluster */

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt
/* $0BFA: receive the file name */
  GetFileName
/* $0C10: */
  i = find_dir_entry (-1);
  if (i >= 0)
  {
    fd_status.bits.FILE_FOUND = 1;		/* file of old name exists */
    if (debuf1.attr)
    {
      fd_status.bits.FILE_READ_ONLY = 1;	/* file is write protected */
    }
    else
    {
/* $0C27: free the directory entry */
      x = big_endian_word ((byte*) &debuf1.ublk);
      memset ((void*) &debuf1, 0, SIZE_DIR_ENTRY);
      write_dir_entry ((word) i);
      CheckFdError
/* $0C43: free the FAT chain */
      free_fat_chain (x);
    }
  }
  CheckFdError
  finish1 ();
}


/* $0C82: search the 'secbuf' for the last non-zero byte (which is assumed
   to be an 'EOF_CHAR') */
byte* end_of_record (void)
{
  byte *ptr;

  ptr = secbuf+SIZE_RECORD-1;
  while (!*ptr)
  {
    if (ptr == secbuf)
    {
      return (byte*) 0;	/* the buffer contains only zeros */
    }
    --ptr;
  }
  return ptr;		/* a non-zero byte found */
}


/* $0CAC: search the directory for the file name in the 'debuf2',
   returns the index of matching directory entry,
   or -1 when no matching directory entry was found,
   or -2 when an EEPROM access error occurred,
   also the procedure looks for the first free directory entry, result stored
   in the 'dir_index' variable */
int find_dir_entry (int x /* directory entry excluded from the search */)
{
  int i;
  for (i=0; i<DIR_ENTRIES; ++i)
  {
    if (i == x)
    {
      continue;
    }
    read_dir_entry ((word) i);
    if (fd_error.all)
    {
      return -2; /* scan prematurely terminated due to EEPROM access error */
    }
    if (debuf1.type)
    {
/* occupied directory entry */
      if (!memcmp ((void*) &debuf1.name, (void*) &debuf2.name, SIZE_FILE_NAME))
      {
        return i;	/* found matching directory entry */
      }
    }
    else
    {
/* free directory entry */
      if (dir_index < 0)	/* skip if free dir. entry already found */
      {
        dir_index = i;	/* first free directory entry */
      }
    }
  }
  return -1;	/* no matching directory entry found */
}


/* $0D6C: search the FAT for a free cluster starting from 'x' */
word find_free_clus (word x)
{
  byte *ptr;
  word y;

  y = x;
  ptr = secbuf+SIZE_FAT_CHUNK*SIZE_FAT_ENTRY;
  do {
    if (ptr > secbuf+SIZE_FAT_CHUNK*SIZE_FAT_ENTRY-1)
    {
      read_fat_entries (x, SIZE_FAT_CHUNK);
      if (fd_error.all)
      {
        return 0;	/* EEPROM access error */
      }
      ptr = secbuf;
    }
    if (!(*ptr & 0x80))
    {
      return x;		/* found */
    }
    ptr += SIZE_FAT_ENTRY;
    if (++x == max_cluster)
    {
/* continue from the beginning of the FAT */
      x = START_DATA/SIZE_BLOCK;
      ptr = secbuf+SIZE_FAT_CHUNK*SIZE_FAT_ENTRY;
    }
  } while (x != y);
  fd_error.bits.DISK_FULL = 1;
  return 0;		/* not found */
}


/* $0DF8: command $60 - rename file */
void fd_cmd_60 (void)
{
  int i, y;
  byte save_type;	/* ($009B) */
  byte save_ublk;	/* ($0098) */
  byte save_lblk;	/* ($0099) */
  byte save_attr;	/* ($0095) */

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt
/* $0E18: receive the old file name */
  GetFileName
  i = find_dir_entry (-1);
  save_type = debuf1.type;
  save_ublk = debuf1.ublk;
  save_lblk = debuf1.lblk;
  save_attr = debuf1.attr;
  GetByteAndCnt
  GetByteAndCnt
/* $0E87: receive the new file name */
  GetFileName
  if (i >= 0)
  {
    fd_status.bits.FILE_FOUND = 1;		/* file of old name exists */
/* $0E99: if the attribute field of the new file name contains $FF, then
   the old file attribute will be preserved */
    if (debuf2.attr == 0xFF && (debuf2.attr = save_attr))
/* the second term is actually an assignment, not a comparison! */
    {
      fd_status.bits.FILE_READ_ONLY = 1;	/* file is write protected */
    }
    else
    {
/* $0EAA: copy the file type and starting cluster from the old file to the
   new one */
      debuf2.ublk = save_ublk;
      debuf2.lblk = save_lblk;
      debuf2.type = save_type;
      if ((y = find_dir_entry (i)) >= 0)
      {
/* $0EC4: file of the new name already exists */
        fd_status.bits.RENAME_FAILED = 1;
      }
      else if (y == -1)
      {
/* $0EDF: file of the new name doesn't exist */
        memcpy ((void*) &debuf1, (void*) &debuf2, SIZE_DIR_ENTRY);
        write_dir_entry ((word) i);
      }
/* otherwise an EEPROM access error occurred */
    }
  }
  CheckFdError
  finish1 ();
}


/* $0F02: command $0x - read a directory entry */
void fd_cmd_0x (void)
{
  int i, step;

  switch (fd_command)
  {
    case 0x00:		/* read the first directory entry */
      i = -1;
      step = 1;
      break;
    case 0x01:		/* read the next directory entry */
      i = dir_index;
      step = 1;
      break;
    default:		/* read the previous directory entry */
      i = dir_index;
      step = -1;
  }
  while ((i += step) >= 0 && i < DIR_ENTRIES)
  {
    read_dir_entry ((word) i);
    CheckFdError
    if (debuf1.type)
    {
/* $0F9D: found an occupied directory entry */
      dir_index = i;
      fd_status.bits.FILE_FOUND = 1;
      transfer (0);
      CheckRequest
      transfer (0x11);		/* number of returned bytes, LSB */
      CheckRequest
      transfer (0x00);		/* number of returned bytes, MSB */
      CheckRequest
      put_byte (fd_status.all);
/* $0FD6: transfer the directory entry */
      put_block ((byte*) &debuf1, SIZE_DIR_ENTRY);
      return;
    }
  }
/* $0F98: out of the directory area, no occupied directory entry found */
  fd_status.bits.END_OF_FILE = 1;
  finish1 ();
}


/* $1028: command $1x - write to a file */
void fd_cmd_1x (void)
{
  DIR_ENTRY *myfcb;
  byte *ptr;
  word x;	/* next cluster in chain */
  word y;
  word z;	/* current cluster */
  byte var88;	/* destination sector in the destination cluster ($0088) */
  byte var95;	/* access mode the file was opened for ($0095) */
  byte var9c;	/* last sector in the last cluster ($009C) */
  byte flag;

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt
  fd_handle = pspdata;
  myfcb = &fcb_table[fd_handle];
  x = big_endian_word ((byte*) &myfcb->ublk);

/* $105F: */
  if (!myfcb->type)
  {
/* $1067: file not opened */
    fd_status.bits.FILE_NOT_OPENED = 1;
    if (transf_and_cnt (0))
    {
      finish1 ();
      return;
    }
  }
  else
  {
/* $1073: file opened */
    if (transf_and_cnt (0))
    {
      finish1 ();
      return;
    }
/* $1080: */
    var95 = access[fd_handle];
  }

/* $1095: */
  if (fd_command & 0x01)
  {
/* $109B: random writing */
    if (!(var95 & 0x01))	/* skip if file opened for random access */
    {
      fd_status.bits.ACCESS_ERROR = 1;
    }
/* $10A0: */
    y = (word) pspdata;			/* record, LSB */
    GetByteAndCnt
    y |= ((word) pspdata) << 8;		/* record, LSB */
    --y;			/* the records are numbered from 1 up */
/* $10B2: */
    if (transf_and_cnt (0))
    {
      finish1 ();
      return;
    }
/* $10BA: */
    if (fd_status.all)
    {
/* $10BE: illegal access mode, random writing to a file opened for sequential
   access */
      get_record ();
      finish1 ();
      return;
    }
/* $10C7: random writing to a file opened for random access */
    var88 = ((byte) y) % SIZE_BLOCK;	/* which sector in the cluster */
    y /= SIZE_BLOCK;			/* number of clusters to skip */
/* $10E2: this loop skips the requested number of clusters */
    do {
      z = x;
      x = read_fat_entry (z);
      CheckFdError
      if (x & 0x4000)
      {
/* $110C: end of the FAT chain encountered */
        var9c = last_sectors (x);
        read_sec1 (z, var9c);
        CheckFdError
        ptr = end_of_record ();
        if (!ptr || *ptr != EOF_CHAR)
        {
/* $113A: */
          fd_status.bits.DATA_ERROR = 1;
          get_record ();
          finish1 ();
          return;
        }
/* $1145: */
        *ptr = 0;	/* clear the End Of File mark 'EOF_CHAR' */
        flag = 0;
        while (y || var88 != var9c)
        {
/* $1177: this loop extends the file to the position being written */
          write_sec1 (z, var9c);
          CheckFdError
          if (++var9c > SIZE_BLOCK-1)
          {
            var9c = 0;
            --y;
            x = find_free_clus (z);
            CheckFdError
            write_fat_entry (z, x | 0x8000 /* used entry */);
            CheckFdError
            write_last_fat_entry (x, 0);
            CheckFdError
            z = x;
            flag = 0;	/* because 'find_free_cluster' uses 'secbuf' */
          }
          if (!flag)
          {
            memset ((void*) secbuf, 0, SIZE_RECORD);
            flag = -1;
          }
        }
        if (flag)
        {
          write_last_fat_entry (x, var9c);
          CheckFdError
        }
        break;		/* y=0, append at the end of file */
      }
    } while (y--);
/* $10F1: specified cluster found before the end of file, y = -1 */
  }

  else
  {
/* $1391: sequential writing */
    if (fd_status.all)
    {
      get_record ();
      finish1 ();
      return;
    }
    if (var95 & 0x03)
    {
/* $139D: sequential writing to a file opened for random access or for input */
      fd_status.bits.ACCESS_ERROR = 1;
      get_record ();
      finish1 ();
      return;
    }
/* $13AA: sequential writing to a file opened for sequential access */
    x = z = find_last_clus (x);
    CheckFdError
    var88 = last_sectors (z);
    y = 0;
  }

  if (y)
  {
/* $1430: writing before the end of the file, read-modify-write operation */
    read_sec1 (z, var88);
    CheckFdError
/* $143D: receive a record from the host and write it to the sector 'z' */
    get_record ();
    write_sec1 (z, var88);
    CheckFdError
  }
  else
  {
/* appending at the end of the file */
    y = fd_counter;
    x = z;		/* not really neccessary */
    if (var88 == 3 && y >= SIZE_RECORD-1)
    {
/* $13FE: another cluster needs to be allocated for the terminating record
   containing only an 'EOF_CHAR' */
      x = find_free_clus (z);
      CheckFdError
    }
/* $1425: -> $12BA: */
    memset ((void*) secbuf, 0, SIZE_RECORD);
    ptr = get_record ();
    if (y < SIZE_RECORD-1)
    {
      *ptr = EOF_CHAR;
    }
    write_sec1 (z, var88);
    CheckFdError
    if (y >= SIZE_RECORD-1)
    {
      clear_secbuf ();
      ++var88;
      var88 %= SIZE_BLOCK;
      write_sec1 (x, var88);
      CheckFdError
/* allocate the new sector in the FAT */
      if (var88)
      {
/* sector in the same cluster */
        write_last_fat_entry (z, var88);
      }
      else
      {
/* sector in a new cluster */
        x &= 0x01FF;
        write_fat_entry (z, x | 0x8000 /* used entry */);
        CheckFdError
        write_last_fat_entry (x, 0);
      }
      CheckFdError
    }
  }

  finish1 ();
}


/* $1483: command $2x - read from a file */
void fd_cmd_2x (void)
{
  DIR_ENTRY *myfcb;
  byte *ptr;
  word x;	/* next cluster in chain */
  word y;
  word z;	/* current cluster */
  byte var88;	/* destination sector in the destination cluster ($0088) */
  byte var9c;	/* last sector in the last cluster ($009C) */

  get_num_bytes ();
  GetByteAndCnt
  GetByteAndCnt
  fd_handle = pspdata;
  myfcb = &fcb_table[fd_handle];
  if (!myfcb->type)
  {
    fd_status.bits.FILE_NOT_OPENED = 1;
    finish1 ();
    return;
  }

  if (fd_command & 0x01)
  {
/* $14DD: random reading */
    if (!(access[fd_handle] & 0x01))
    {
/* $14E0: file not opened for random access */
      fd_status.bits.ACCESS_ERROR = 1;
      finish1 ();
      return;
    }
/* $14E5: */
    GetByteAndCnt
    y = (word) pspdata;			/* record, LSB */
    GetByteAndCnt
    y |= ((word) pspdata) << 8;		/* record, LSB */
    --y;			/* the records are numbered from 1 up */
    var88 = ((byte) y) % SIZE_BLOCK;	/* which sector in the cluster */
    y /= SIZE_BLOCK;			/* number of clusters to skip */
    x = big_endian_word ((byte*) &myfcb->ublk);
/* $150F: this loop skips the requested number of clusters */
    do {
      z = x;
      x = read_fat_entry (z);
      CheckFdError
      if (x & 0x4000)
      {
/* $1534: end of the FAT chain encountered */
        if (y || var88 >= last_sectors (x))
        {
/* $154E: attempt to read the terminating record or past the end of file */
          fd_status.bits.DATA_ERROR = 1;
          finish1 ();
          return;
        }
      }
    } while (y--);
/* $151E: specified cluster found */
  }

  else
  {
/* $1553: sequential reading */
    z = position[fd_handle];
    var88 = last_sectors (z);
    x = read_fat_entry (z);
    CheckFdError
    if (x & 0x4000)		/* end of chain? */
    {
/* $1574: last cluster of the file */
      var9c = last_sectors (x);
      if (var88 > var9c)
      {
/* $158B: attempt to read past the end of file */
        fd_status.bits.END_OF_FILE = 1;
        finish5 (0);
        return;
      }
      if (var88 == var9c)
      {
/* $1587: last record of the file */
        fd_status.bits.END_OF_FILE = 1;
      }
    }
/* $1590: advance the position */
    if (!((position[fd_handle] += 0x1000) & 0x3000))	/* next sector */
    {
      position[fd_handle] = x & 0x01FF;			/* next cluster */
    }
  }

/* $15D6: */
  read_sec1 (z, var88);
  CheckFdError
  if (!fd_status.bits.END_OF_FILE)
  {
/* $1649: not the last record of the file */
    y = SIZE_RECORD;
  }
  else
  {
/* $15EC: last record of the file */
    ptr = end_of_record ();
    y = (ptr) ? ptr-secbuf : 0;
  }
  finish5 (y);
}


/* $15FB: common closing sequence */
void finish5 (word x /* data block size */)
{
  transfer (0);
  CheckRequest
  ++x;					/* account for the status byte */
/* $160F: transmit the number of returned bytes to the host */
  transfer ((byte) x);			/* LSB */
  CheckRequest
  transfer ((byte) (x >> 8));		/* MSB */
  CheckRequest
/* $162A: transmit the status code followed by the data block to the host */
  put_byte (fd_status.all);
  --x;
  put_block (secbuf, x);
}


/* $1690: command $40 - close file */
void fd_cmd_40 (void)
{
  transfer (0);
  fd_handle = pspdata;
  if (fd_handle & 0x80)
  {
    close_all ();		/* close all files */
  }
  else
  {
    close_file ();		/* close a single file */
  }
  put_byte (fd_error.all);
}


/* $16C0: close a file */
void close_file (void)
{
/* clear the File Control Block */
  fcb_table[fd_handle].type = 0;
/* $16E6: clear the record index */
  position[fd_handle] = 0;
}


/* close all files */
void close_all (void)
{
  for (fd_handle=0; fd_handle<MAX_FILES; ++fd_handle)
  {
    close_file ();
  }
}


/* $1726: command $C0 - get the file size (number of records) */
void fd_cmd_c0 (void)
{
  DIR_ENTRY *myfcb;
  word i, x;

  transfer (0);
  fd_handle = pspdata;
  myfcb = &fcb_table[fd_handle];
  if (!myfcb->type)
  {
/* $1741: file not opened (file type = 0) */
    transfer (0);
    fd_status.bits.FILE_NOT_OPENED = 1;
    put_byte (fd_status.all);
  }
  else
  {
/* $1749: file opened (file type <> 0) */
    i = big_endian_word ((byte*) &myfcb->ublk);
    x = 0;
/* $175C: this loop follows the FAT chain counting the number of clusters
   occupied by the file */
    while (!((i = read_fat_entry (i)) & 0x4000) && !fd_error.all)
    {
      x += SIZE_BLOCK;
    }
/* $1797: end of chain */
    x += (word) last_sectors (i);
/* $17A8: the terminating record isn't included when a file is opened for
   random access */
    if (!(access[fd_handle] & 0x01))
    {
      ++x;
    }
/* The original MD-100 code ignores 'fd_error', because it uses a FAT copy in
   the RAM, so it doesn't read anything from the floppy disk.
   Let's report the EEPROM access errors with some 'fd_status' bit. */
    if (fd_error.all)
    {
      fd_status.bits.DATA_ERROR = 1;
    }
    finish2 (x);
  }
}


/* returns the number of occupied sectors in the last cluster - 1 specified
   by bits 5,4 of the upper byte of the last FAT entry
   (replaces repeated code blocks: $095B..$0966, $1797..$17A4) */
byte last_sectors (word lastfatentry)
{
  byte x;

  x = 0;
  if (lastfatentry & 0x2000)
  {
    x |= 0x02;
  }
  if (lastfatentry & 0x1000)
  {
    x |= 0x01;
  }
  return x;
}


/* $17B7: transfer 'fd_status.all' and the word 'x' to the host */
void finish2 (word x)
{
  transfer (0);
  CheckRequest
  transfer (fd_status.all);
  CheckRequest
  transfer ((byte) x);		/* LSB */
  CheckRequest
  put_byte ((byte) (x>>8));	/* MSB */
}


/* $17D5: command $D0 - get the free EEPROM space (number of free clusters) */
void fd_cmd_d0 (void)
{
  byte *ptr;
  word i, x;

  x = 0;
  ptr = secbuf+SIZE_RECORD;
  for (i=0; i<max_cluster; ++i)
  {
    if (ptr > secbuf+SIZE_RECORD-1)
    {
      read_fat_entries (i, SIZE_RECORD/SIZE_FAT_ENTRY);
      CheckFdError
      ptr = secbuf;
    }
    if (!(*ptr & 0x80))
    {
/* FAT entry marked as free, increment the number of free clusters */
      ++x;
    }
    ptr += SIZE_FAT_ENTRY;
  }
  finish2 (x);
}
