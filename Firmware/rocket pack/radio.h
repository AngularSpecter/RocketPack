#ifndef RADIO_h
#define RADIO_h

#include <msp430FR5739.h>
#include "types.h"


#define STATUS  BIT4
#define SLEEP   BIT5
#define CONFIG  BIT6
#define RESET	BIT7

/*packet formatting:

NSEL       :   [NSEL] [address] [0x00]
CAL        :   [CAL] [0x00]
DATA       :   [DATA] [sensor mask] [Timeout] [0x00]
HOSTDISC   :   [HOSTDISC] [0x00]

*/

#define HOSTDISC   0x01   //Broadcast by the host to the base station to announce its presence.
#define CMD        0x02   //Base -> node: command for remote node to execute
                          //node -> base: return of command
#define CAL        0x03   //Base -> node: Request calibration data
                          //node -> base: Return calibration data
#define DATA       0x04   //node -> base: Data
#define NSEL       0x05   //base -> node: address of node to be active..all other sleep
#define RADIOACK   0x06   //command ACK
#define RADIONACK  0x07   //command NACK
#define RADIOBOOT  0x08

extern volatile unsigned int radio_command_mode;

/** Binary Mode definitions ************************************************************************************************************************/
#define AT 0x05 // (5d) Guard Time After 0x02 – 0xFFFF [x 100 msec] Command Mode Options 2 0x0A (10d)
#define BD 0x15 // (21d) Interface Data Rate Standard baud rates: 0 – 6  Non-standard baud rates: 0x7D – 0xFFFF Serial Interfacing 2 0x03 9600bps
#define BT 0x04 // (4d) Guard Time Before 2 – 0xFFFF [x 100 msec] Command Mode Options 2 0x0A (10d)
#define CC 0x13 // (19d) Command Sequence Character 0x20 – 0x7F Command Mode Options 1 0x2B (“+”)
#define CD 0x28 // (40d) DO3 Configuration 0 - 4 Serial Interfacing 1 0
#define CN 0x09 // (9d) Exit AT Command Mode - Command Mode Options - -
#define CS 0x1F // (31d) DO2 Configuration 0 – 4 Serial Interfacing 1 0
#define CT 0x06 // (6d) Command Mode Timeout 0x02 – 0xFFFF [x 100 msec] Command Mode Options 2 0xC8 (200d)
#define DT 0x00 // (0d) Destination Address 0 – 0xFFFF Networking 2 0
#define E0 0x0A // (10d) Echo Off - Command Mode Options - -
#define E1 0x0B // (11d) Echo On - Command Mode Options - -
#define ER 0x0F // (15d) Receive Error Count 0 – 0xFFFF Diagnostics 2 0
#define FH 0x0D // (13d) Force Wake-up Initializer - Sleep (Low Power) - -
#define FL 0x07 // (7d) Software Flow Control 0 – 1 Serial Interfacing 1 0
//      FR		//  Force Module to reset
#define FT 0x24 // (36d) Flow Control Threshold 0 – (DI buffer – 0x11) [bytes] Serial Interfacing 2 varies
#define GD 0x10 // (16d) Receive Good Count 0 – 0xFFFF Diagnostics 2 0
#define HP 0x11 // (17d) Hopping Channel 0 – 6 Networking 1 0
#define HT 0x03 // (3d) Time before Wake-up Initializer 0 – 0xFFFF [x 100 msec] Sleep (Low Power) 2 0xFFFF
#define ID 0x27 // (39d) Module VID User set table: 0x10 - 0x7FFF Read-only: 0x8000 – 0xFFFF Networking 2 -
#define LH 0x0C // (12d) Wake-up Initializer Timer 0 – 0xFF [x 100 msec] Sleep (Low Power) 1 1
#define MD 0x32 // (50d) RF Mode 0 – 4 Networking & Security 1 0
#define MK 0x12 // (18d) Address Mask 0 – 0xFFFF Networking 2 0xFFFF
#define MY 0x2A // (42d) Source Address 0 – 0xFFFF Networking & Security 2 0xFFFF
#define NB 0x23 // (35d) Parity 0 – 5 Serial Interfacing 1 0
#define PC 0x1E // (30d) Power-up Mode 0 – 1 Command Mode Options 1 0
#define PK 0x29 // (41d) RF Packet Size 0 - 0x100 [bytes] Serial Interfacing 2 0x40 (64d)
#define PL 0x3c // (60d) RF Power Level 0-4 (Special) 1 4
#define PW 0x1D // (29d) Pin Wake-up 0 – 1 Sleep (Low Power) 1 0
#define RB 0x20 // (32d) Packetization Threshold 0 - 0x100 [bytes] Serial Interfacing 2 0x01
#define RE 0x0E // (14d) Restore Defaults - (Special) - -
#define RN 0x19 // (25d) Delay Slots 0 – 0xFF [slots] Networking 1 0
#define RO 0x21 // (33d) Packetization Timeout 0 – 0xFFFF [x 200 µsec] Serial Interfacing 2 0
#define RP 0x22 // (34d) RSSI PWM Timer 0 - 0x7F [x 100 msec] Diagnostics 1 0
#define RR 0x18 // (24d) Retries 0 – 0xFF Networking 1 0
#define RS 0x1C // (28d) RSSI 0x06 – 0x36 [read-only] Diagnostics 1 -
#define RT 0x16 // (22d) DI2 Configuration 0 - 2 Serial Interfacing 1 0
#define RZ 0x2C // (44d) DI Buffer Size [read-only] Diagnostics - -
#define SB 0x36 // (54d) Stop Bits 0 - 1 Serial Interfacing 1 0
#define SH 0x25 // (37d) Serial Number High 0 – 0xFFFF [read-only] Diagnostics 2 -
#define SL 0x26 // (38d) Serial Number Low 0 – 0xFFFF [read-only] Diagnostics 2 -
#define SM 0x01 // (1d) Sleep Mode 0, 1, 3 - 8 Sleep (Low Power) 1 0
#define ST 0x02 // (2d) Time before Sleep 0x10 – 0xFFFF [x 100 msec] Sleep (Low Power) 2 0x64 (100d)
#define SY 0x17 // (23d) Time before Initialization 0 – 0xFF [x 100 msec] Networking 1 0 (disabled)
#define TR 0x1B // (27d) Transmit Error Count 0 – 0xFFFF Diagnostics 2 0
#define TT 0x1A // (26d) Streaming Limit 0 – 0xFFFF [0 = disabled] Networking 2 0xFFFF
#define VR 0x14 // (20d) Firmware Version 0 - 0xFFFF [read-only] Diagnostics 2 -

/*** Better ASCII mode definitions  **********************************************************************************************************************/

#define CMD_START		"+++" //Start string to put module in command mode

//Diagnostic
#define ECHO_OFF         "E0" // (10d) Echo Off - Command Mode Options - -
#define ECHO_ON          "E1" // (11d) Echo On - Command Mode Options - -
#define RSSI_PWM         "RP" // (34d) RSSI PWM Timer 0 - 0x7F [x 100 msec] Diagnostics 1 0
#define RSSI             "RS" // (28d) RSSI 0x06 – 0x36 [read-only] Diagnostics 1 -
#define TX_ERR_CNT       "TR" // (27d) Transmit Error Count 0 – 0xFFFF Diagnostics 2 0
#define RX_ERR_CNT       "ER" // (15d) Receive Error Count 0 – 0xFFFF Diagnostics 2 0
#define RX_GOOD_CNT      "GD" // (16d) Receive Good Count 0 – 0xFFFF Diagnostics 2 0

#define FW_VER           "VR" // (20d) Firmware Version 0 - 0xFFFF [read-only] Diagnostics 2 -
#define SN_LOW           "SH" // (37d) Serial Number High 0 – 0xFFFF [read-only] Diagnostics 2 -
#define SN_HIGH          "SL" // (38d) Serial Number Low 0 – 0xFFFF [read-only] Diagnostics 2 -
#define VEND_ID          "ID" // (39d) Module VID User set table: 0x10 - 0x7FFF Read-only: 0x8000 – 0xFFFF Networking 2 -
#define FCTRY_RST        "RE" // (14d) Restore Defaults - (Special) - -

//Networking
#define BAUDRATE         "BD" // (21d) Interface Data Rate Standard baud rates: 0 – 6  Non-standard baud rates: 0x7D – 0xFFFF Serial Interfacing 2 0x03 9600bps
#define ADDRESS_DST      "DT" // (0d) Destination Address 0 – 0xFFFF Networking 2 0
#define CHNL_HOPPING     "HP" // (17d) Hopping Channel 0 – 6 Networking 1 0
#define ADDRESS_MASK 	 "MK" // (18d) Address Mask 0 – 0xFFFF Networking 2 0xFFFF
#define ADDRESS_SRC      "MY" // (42d) Source Address 0 – 0xFFFF Networking & Security 2 0xFFFF

//Power
#define FORCE_WU         "FH" // (13d) Force Wake-up Initializer - Sleep (Low Power) - -
#define FORCE_RST        "FR" //  Force Module to reset

#define RFMODE           "MD" // (50d) RF Mode 0 – 4 Networking & Security 1 0
#define P2P		  0
#define REPEATER  3
#define END_NODE  4

#define WU_MODE          "PC" // (30d) Power-up Mode 0 – 1 Command Mode Options 1 0
#define IDLE      0
#define CMD_MODE  1

#define TX_PWR_LVL       "PL" // (60d) RF Power Level 0-4 (Special) 1 4
#define mW5   0    //+7.0 dBm, (5 mW)
#define mW32  1    //+15.0dBm, (32 mW)
#define mW63  2    //+18.0dBm, (63 mW)
#define mW125 3    //+21.0dBm, (125 mW)
#define mW250 4    //+24.0 dBm, (250 mW)

#define WU_PIN           "PW" // (29d) Pin Wake-up 0 – 1 Sleep (Low Power) 1 0


#define SLP_MODE         "SM" // (1d) Sleep Mode 0, 1, 3 - 8 Sleep (Low Power) 1 0
#define SLP_DISABLED 0 //Disabled
#define SLP_PIN      1 // Pin Sleep
#define SLP_0.5      3 //Cyclic 0.5 second sleep
#define SLP_1        4 // Cyclic 1.0 second sleep
#define SLP_2        5 // Cyclic 2.0 second sleep
#define SLP_4        6 // Cyclic 4.0 second sleep
#define SLP_8        7 // Cyclic 8.0 second sleep
#define SLP_16       8 // Cyclic 16.0 second sleep



void config_radio(void);
void sleep_radio(char mode);
char radio_enter_CMD(void);
char radio_proc_ack(unsigned int * buffer);
char radio_leave_CMD(void);
char radio_set_value(char * reg, char * value);
char radio_read_value(char * reg, unsigned int * value);
char radio_write_changes(void);

unsigned int str2dec(unsigned int * string);
#endif
