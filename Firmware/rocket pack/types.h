#ifndef TYPES_h
#define TYPES_h

#define uint8 	unsigned char
#define int8  	signed char
#define uint16	unsigned int
#define int16	signed int
#define uint32  unsigned long
#define int32	signed long

#define NULL	0x00
#define TRUE    (uint8)1
#define FALSE   (uint8)0

extern volatile unsigned int sensorMask;

typedef enum {
	SLEEPWAIT,
	PRESTAGE,
	SHUTDOWN,
	ACTIVEMODE
}mode;

typedef enum {
	NOERROR,		//everything is fine
	OVERFLOW_NA,	//buffer write overflow detected...buffer write not allowed
	OVERFLOW_WA,	//buffer write overflow detected...buffer write allowed
	BUFFEREMPTY	,	//tried to read an empty buffer.
	INVALIDOPTION,	//Invalid option supplied
	TIMEOUT			//UART operation timed out
}UARTERROR;

typedef struct UART_buffer{
	volatile unsigned int buffer[50];
	volatile unsigned int buffer_len;
	volatile unsigned int read_idx;
	volatile unsigned int write_idx;
	volatile unsigned int bytes_to_proc;
	volatile unsigned int packets_rxd;
	volatile unsigned int last_command_idx;
	volatile unsigned char term_char;
} UART_buffer;

typedef enum {READ, WRITE} dbBuffPos;

typedef enum {
	BUF_NOERROR,		//everything is fine
	BUF_OVERFLOW_NA,	//buffer write overflow detected...buffer write not allowed
	BUF_OVERFLOW_WA,	//buffer write overflow detected...buffer write allowed
	BUF_BUFFEREMPTY	,	//tried to read an empty buffer.
	BUF_INVALIDOPTION	//Invalid option supplied
}BUFFERERROR;


#endif
