// Radial Serial Protocol for TU58 packet flag byte definitions
#define DATA		1
#define CONTROL		2
#define INIT		4
#define BOOT		8
#define CONTINUE	16
#define XON		    17
#define XOFF		19

// command packet opcode definitions
#define NOP_CMD       0   // request end packet
#define INIT_CMD      1   // reset & send end packet
#define READ_CMD      2   // seek and read # of blocks requested
#define WRITE_CMD     3   // seek and write # of blocks requested
#define POSITION_CMD  5   // seek to block requested
#define DIAGNOSE_CMD  7   // run internal diagnostice and send end packet
#define GETSTATUS_CMD 8   // get status, sends end packet
#define SETSTATUS_CMD 9   // set status, sends end packet
#define GETCHAR_CMD  10   // get characteristics -> MRSP supported ?
#define END_CMD    0x40   // end packet from TU58

#define MRSP_SWITCH    8   // setting bit 3 in cmd->switch activates MRSP mode in new TU58 drives

#define BLOCK_SIZE 128     // TU58 block size
//#define PACKET_SIZE 128  // size of data packets, 4 to a standard block

/* decoded octal success codes per pp 3-5 of TU58 manaul
octal binary     dec  hex
  0   00000000     0    0  OK
  1   00000001     1    1  OK with retries
377   11111111    -1   FF  Failed self test
376   11111110    -2   FE  Partial operation (end of medium)
370   11111000    -8   F8  Bad Unit number
367   11110111    -9   F7  No Cartidge
365   11110101    -11  F5  Write Protect
357   11101111    -17  EF  Data check error
340   11100000    -32  E0  Seek error (block not found)
337   11011111    -33  DF  Motor Stopped
320   11010000    -48  D0  Bad Op Code
311   11001001    -55  C9  Bad Block # (ie > 511)
Basically a success code < 0 is bad news!
*/

#define OK          0
#define OKR         1
#define FAIL       -1
#define EOM        -2
#define BAD_UNIT   -8
#define NO_TAPE    -9
#define WRT_PROT   -11
#define BAD_CHECK  -17
#define BAD_SEEK   -32
#define BAD_MOTOR  -33
#define BAD_OPCODE -48
#define BAD_BLOCK  -55

typedef struct _rsp_cmd {
unsigned char opcode, modifier, unit, switches;
unsigned short sequence, count, block;
} RSP_CMD;

