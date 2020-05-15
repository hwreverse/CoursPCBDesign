typedef unsigned char byte;
typedef unsigned short int word;
typedef unsigned long ulong;

/* structure of directory entry */
typedef struct _dir_entry {
  byte type;		/* file type */
  char name[8];		/* file name */
  char ext[3];		/* file name extension */
  char unused;
  byte ublk;		/* number of the starting cluster, MSB */
  byte lblk;		/* number of the starting cluster, LSB */
  byte attr;		/* file attribute */
} DIR_ENTRY;

/* function prototypes */
void high_vector (void);
void high_isr (void);
byte ee_presence (byte control);
void ee_scan (void);
byte ee_read (byte control, word address, byte *rdptr, byte length);
byte ee_write (byte control, word address, byte *wrptr, byte length);
byte ee_cntrl (word x);
void read_sector (word x);
void write_sector (word x);
void read_sec1 (word cluster, byte sector);
void write_sec1 (word cluster, byte sector);
void read_dir_entry (word x);
void write_dir_entry (word x);
word read_fat_entry (word x);
void read_fat_entries (word x, byte i);
void write_fat_entry (word x, word y);
void write_last_fat_entry (word cluster, byte sector);
void get_num_bytes (void);
byte get_block (byte *ptr, word size);
byte put_block (byte *ptr, word size);
byte* get_record (void);
word find_last_clus (word x);
void free_fat_chain (word x);
void wait_for_rd_mode (void);
void fd_cmd_90 (void);
void put_byte (byte x);
void transfer (byte x);
void fd_cmd_80 (void);
void fd_cmd_70 (void);
byte transf_and_cnt (byte x);
void fd_cmd_3x (void);
void finish3 (word x);
void create_file (void);
void clear_secbuf (void);
void finish1 (void);
void fd_cmd_50 (void);
byte* end_of_record (void);
int find_dir_entry (int x);
word find_free_clus (word x);
void fd_cmd_60 (void);
void fd_cmd_0x (void);
void fd_cmd_1x (void);
void fd_cmd_2x (void);
void finish5 (word x);
void fd_cmd_40 (void);
void close_file (void);
void close_all (void);
void fd_cmd_c0 (void);
byte last_sectors (word lastfatentry);
void finish2 (word x);
void fd_cmd_d0 (void);
