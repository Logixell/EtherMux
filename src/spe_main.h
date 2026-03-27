#include <stdint.h>
#include "ssd1306.h"

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
int spi_write_array(uint8_t *data, size_t length);
int spi_read_register16(int address, int *data);
int config_fpga();
void display(ssd1306_t *disp);
void draw_icon(ssd1306_t *p, uint8_t x_offset, uint8_t y_offset, uint8_t width, uint8_t height, const uint8_t *icon_data);
int comm_try_receive_char(uint8_t *out_char, uint8_t com_port);
int comm_try_receive_line(uint8_t com_port);
int get_hop();

// FPGA Interface Command Defines
#define FPGA_CMD_READ_REG 1
#define FPGA_CMD_WRITE_REG 2
#define FPGA_CMD_WRITE_TX_FIFO 3
#define FPGA_CMD_READ_RX_FIFO 4

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
#define FPGA_DC_KP 22
#define FPGA_DC_KI 23
#define FPGA_DC_KD 24
#define FPGA_PACKET_GEN 25
#define FPGA_SPI_RD 26
#define FPGA_HOP 31

#define BIT_0_MASK 0x01 // Bit 0 mask
#define BIT_1_MASK 0x02 // Bit 1 mask
#define BIT_2_MASK 0x04 // Bit 2 mask
#define BIT_3_MASK 0x08 // Bit 3 mask
#define BIT_4_MASK 0x10 // Bit 4 mask
#define BIT_5_MASK 0x20 // Bit 5 mask
#define BIT_6_MASK 0x40 // Bit 6 mask
#define BIT_7_MASK 0x80 // Bit 7 mask

#endif // SPE_MAIN_H