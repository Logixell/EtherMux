#ifndef SPE_MAIN_H
#define SPE_MAIN_H

#define CLI_ECHO 1 //1= echo characters to the terminal, 0= do not echo characters to the terminal
#define FPGA_INTERFACE_TYPE 1  // 1=SPI or 2=UART Interface to FPGA

// global variables
extern int scroll;

// Function declarations
int read_register8(int address, int *data);
int read_register16(int address, int *data );
int write_register8(int address, int data);
int spi_read_register8(int address, int *data);  
int spi_write_register8(int address, int data);
int spi_read_register16(int address, int *data);

// FPGA Interface Command Defines
#define FPGA_CMD_READ_REG 1
#define FPGA_CMD_WRITE_REG 2
#define FPGA_CMD_WRITE_TX_DPRAM 3
#define FPGA_CMD_READ_TX_DPRAM 4
#define FPGA_CMD_WRITE_RX_DPRAM 5
#define FPGA_CMD_READ_RX_DPRAM 6
// Ethernet Phy
#define FPGA_PHYA_ADDR 0
#define FPGA_PHYB_ADDR 8

// FPGA resister addresses
#define FPGA_VERSION 0
#define FPGA_CTRL 1
#define FPGA_SENT_LSB 11
#define FPGA_SENT_MSB 12
#define FPGA_SENT_ROT_LSB 13
#define FPGA_SENT_ROT_MSB 14
#define FPGA_SMI_ADDR 2
#define FPGA_SMI_DATA_LSB 3
#define FPGA_SMI_DATA_MSB 4
#define FPGA_SMI_CTL_ADDR 5
#define FPGA_ADC1 6
#define FPGA_ADC2 7
#define FPGA_ADC3 8
#define FPGA_ADC4 9
#define FPGA_DC_ROT_LSB 20
#define FPGA_DC_ROT_MSB 21
#endif // SPE_MAIN_H