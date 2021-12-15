#ifndef SPI_SCC_HG
#define SPI_SCC_HG

/***************************************************/
/**\name    REGISTER ADDRESS DEFINITION  */
/***************************************************/
#define REQ_READ_RATE  0x040000f7
#define REQ_READ_ACC_X  0x100000e9
#define REQ_READ_ACC_Y  0x140000ef
#define REQ_READ_ACC_Z  0x180000e5
#define REQ_READ_TEMP  0x1c0000e3
#define REQ_WRITE_FLT_60 0xfc200006
#define REQ_WRITE_FLT_10 0xfc1000c7
#define REQ_READ_STAT_SUM  0x7c0000b3
#define REQ_READ_RATE_STAT1 0x240000c7
#define REQ_READ_RATE_STAT2 0x280000cd
#define REQ_READ_ACC_STAT 0x3c0000d3
#define REQ_READ_COM_STAT1 0x6c0000ab
// Special requests for SCC2130
#define REQ_HARD_RESET 0xD8000431
// Frame field masks for SCC2130
#define OPCODE_FIELD_MASK 0xFC000000
#define RS_FIELD_MASK 0x03000000
#define DATA_FIELD_MASK 0x00FFFF00
#define CRC_FIELD_MASK 0x000000FF

typedef struct Output_scc
{
/// \brief store scc2130 data
/// \returns a struct with the scc2130 temperature and gyro data.
    signed int gyro;
    signed int temp;
} Output_scc;

typedef struct Status_scc
{
    uint16_t StatSum;
    uint16_t RateStat1;
    uint16_t RateStat2;
    uint16_t AccStat;
    uint16_t ComStat1;
}Status_scc;


/***************************************************/
/**\name    CONFIGURE SSI FUNCTIONS */
/***************************************************/
void ConfigureSSI0(void);
void ConfigureSSI1(void);
void ConfigureSSI2(void);

uint32_t send_request_SCC1(uint32_t request);
uint32_t send_request_SCC2(uint32_t request);
uint32_t send_request_SCC3(uint32_t request);
uint32_t send_request_SCC4(uint32_t request);
uint32_t send_request_SCC5(uint32_t request);
uint32_t send_request_SCC6(uint32_t request);

Output_scc read_and_process_gyro_SCC1(void);
Output_scc read_and_process_gyro_SCC2(void);
Output_scc read_and_process_gyro_SCC3(void);
Output_scc read_and_process_gyro_SCC4(void);
Output_scc read_and_process_gyro_SCC5(void);
Output_scc read_and_process_gyro_SCC6(void);

void init_scc2130(void);

void delayMs(uint32_t ui32Ms);

Status_scc read_scc_status(int scc);

#endif
