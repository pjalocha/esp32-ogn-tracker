#define CMD_SETSLEEP                    0x84  // [1] set sleep
#define CMD_SETSTANDBY                  0x80  // [1] set standby
#define CMD_SETFS                       0xC1  // [0] test mode: set synthesis
#define CMD_SETTX                       0x83  // [3] set transmitter mode, timeout [15.625us] ?
#define CMD_SETRX                       0x82  // [3] set receive mode, timeout
#define CMD_STOPTIMERONPREAMBLE         0x9F  // [1]
#define CMD_SETRXDUTYCYCLE              0x94  // [6] sleep/receive cycle to save power
#define CMD_SETCAD                      0xC5  // [0]
#define CMD_SETTXCONTINUOUSWAVE         0xD1  // [1] test mode: continues wave
#define CMD_SETTXINFINITEPREAMBLE       0xD2  // [1] test mode: continues preamble
#define CMD_SETREGULATORMODE            0x96  // [1] 0 = LDO, 1=DC-DC + LDO
#define CMD_CALIBRATE                   0x89  // [1]
#define CMD_CALIBRATEIMAGE              0x98  // [2]
#define CMD_SETPACONFIG                 0x95  // [4] PA control: duty
#define CMD_SETRXTXFALLBACKMODE         0x93  // [1]

#define CMD_WRITEREGISTER               0x0D  // write one or more registers
#define CMD_READREGISTER                0x1D  // read one or more registers
#define CMD_WRITEBUFFER                 0x0E  // write the packet
#define CMD_READBUFFER                  0x1E  // read the packet

#define CMD_SETDIOIRQPARAMS             0x08  // [8] IRQ enable and masks for DIO1/2/3 outputs
#define CMD_GETIRQSTATUS                0x12  // [stat+2] get IRQ flags
#define CMD_CLEARIRQSTATUS              0x02  // [2] clear IRQ flags according to given mask
#define CMD_SETDIO2ASRFSWITCHCTRL       0x9D  // [1] Use DIO2 as the RF switch control
#define CMD_SETDIO3ASTCXOCTRL           0x97  // [4] Use DIO3 as TCXO control: voltage, delay

#define CMD_SETRFFREQUENCY              0x86  // [4] frequency [Fxtal/2^25]
#define CMD_SETPACKETTYPE               0x8A  // [1] 0x00=GFSK, 0x01=LoRa (must be the first configuration command)
#define CMD_GETPACKETTYPE               0x11
#define CMD_SETTXPARAMS                 0x8E  // [2] TxPower: -17..+14 or -9..+22 [dBm] depending on PA config RampTime: 0..7: 10,20,40,80,200,800,1700,3400us
#define CMD_SETMODULATIONPARAMS         0x8B  // [8] depends on the protocol: FSK or LoRa
#define CMD_SETPACKETPARAMS             0x8C  // [9]
#define CMD_SETCADPARAMS                0x88
#define CMD_SETBUFFERBASEADDRESS        0x8F  // [2] Tx-base address, Rx-base address
#define CMD_SETLORASYMBNUMTIMEOUT       0xA0  // [1] timeout [symbols]

#define CMD_GETSTATUS                   0xC0
#define CMD_GETRSSIINST                 0x15  // [stat+1] RX-RSSI at this moment
#define CMD_GETRXBUFFERSTATUS           0x13  // [stat+2] RX-packet: length, buffer pointer
#define CMD_GETPACKETSTATUS             0x14  // [stat+3] RX-packet: RSSI/SNR
#define CMD_GETDEVICEERRORS             0x17
#define CMD_CLEARDEVICEERRORS           0x07
#define CMD_GETSTATS                    0x10  // [stat+6] RX-statistics: packets, errors
#define CMD_RESETSTATS                  0x00  // [stat+6] RX-statistics reset

// registers
#define REG_WHITENINGMSB        0x06B8
#define REG_WHITENINGLSB        0x06B9
#define REG_CRCINITVALMSB       0x06BC
#define REG_CRCINITVALLSB       0x06BD
#define REG_CRCPOLYVALMSB       0x06BE
#define REG_CRCPOLYVALLSB       0x06BF
#define REG_SYNCWORD0           0x06C0
#define REG_SYNCWORD1           0x06C1
#define REG_SYNCWORD2           0x06C2
#define REG_SYNCWORD3           0x06C3
#define REG_SYNCWORD4           0x06C4
#define REG_SYNCWORD5           0x06C5
#define REG_SYNCWORD6           0x06C6
#define REG_SYNCWORD7           0x06C7
#define REG_NODEADDRESS         0x06CD
#define REG_BROADCASTADDR       0x06CE
#define REG_LORASYNCWORD        0x0740
#define REG_LORASYNCWORDMSB     0x0740
#define REG_LORASYNCWORDLSB     0x0741
#define REG_RANDOMNUMBERGEN0    0x0819
#define REG_RANDOMNUMBERGEN1    0x081A
#define REG_RANDOMNUMBERGEN2    0x081B
#define REG_RANDOMNUMBERGEN3    0x081C
#define REG_RXGAIN              0x08AC
#define REG_OCPCONFIG           0x08E7
#define REG_XTATRIM             0x0911
#define REG_XTBTRIM             0x0912

// IRQ types
#define IRQ_TXDONE              (1 << 0)
#define IRQ_RXDONE              (1 << 1)
#define IRQ_PREAMBLEDETECTED    (1 << 2)
#define IRQ_SYNCWORDVALID       (1 << 3)
#define IRQ_HEADERVALID         (1 << 4)
#define IRQ_HEADERERR           (1 << 5)
#define IRQ_CRCERR              (1 << 6)
#define IRQ_CADDONE             (1 << 7)
#define IRQ_CADDETECTED         (1 << 8)
#define IRQ_TIMEOUT             (1 << 9)
#define IRQ_ALL                 0x3FF

