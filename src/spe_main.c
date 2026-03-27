/*  
    EtherMux Open Source Protocol for Single Pair Ethernet 

    Copyright (c) 2026 Thomas Gsell Ethermux.com 
    This file may be distributed under the terms of the GNU GPL-3.0 license.
*/


#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
//#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "blink.pio.h"
#include "spe_cli.h"
#include "spe_main.h"
#include "hardware/i2c.h" // Add this line for I2C support
#include "spe_config.h"
#include "ssd1306.h"
#include "fpgabitstream_V0_1.h"  // FPGA bitstream
//Manually edit bitstream.h to add "extern const" this places the bitstream into flash instead of RAM : extern const unsigned char fpga_config[];


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16 //GP16=pin 21
#define PIN_CS   17 //GP17=pin 22
//      GND           pin 23
#define PIN_SCK  18 //GP18=pin 24
#define PIN_MOSI 19 //GP19=pin 25

#define LED_PIN 25 // On-board LED

//FPGA config (also uses spi0 above)
#define PIN_FPGA_PROG  22 //GP22=pin 
#define PIN_FPGA_DONE   24 //GP24=pin   --BUG not connected on v0.1 board

//I2C0 pins (Qwiic)
#define QWIIC_PORT i2c0
#define QWIIC_SDA 4 // GP4=pin 6
#define QWIIC_SCL 5 // GP5=pin 7
#define MODULINO_KNOB_ADDR 0XB0 //I2C address of Modulino knob 0X76 + 0X3A

// I2C1 pins (Display)
#define I2C1_SDA 6 // GP6 = pin 
#define I2C1_SCL 7 // GP7 = pin 

#define MAX_TOKENS 10
#define MAX_TOKEN_LENGTH 50

#define MAX_COMM_BUFFER_SIZE 256

char comm_buffer[2][MAX_COMM_BUFFER_SIZE];
int comm_buffer_index[2] = {0, 0};


// empty square icon 16x16
const uint8_t icon_data_empty[] = { //No link partner found
    0xFC, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0xFC, 0x00,
    0x3F, 0x40, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,   0x80, 0x80, 0x80, 0x80, 0x80, 0x40, 0x3F, 0x00
};
const uint8_t icon_data_link[] = { //Link established
    0xFC, 0x02, 0x81, 0x61, 0x01, 0x01, 0x01, 0x01,   0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0xFC, 0x00,
    0x3F, 0x40, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,   0x80, 0x80, 0x80, 0x86, 0x81, 0x41, 0x3F, 0x00
};
const uint8_t icon_data_init[] = { //Discovery Mode
    0xFC, 0x02, 0x81, 0x61, 0x01, 0x81, 0x01, 0x01,   0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0xFC, 0x00,
    0x3F, 0x40, 0x80, 0x80, 0x80, 0x8F, 0x80, 0x80,   0x80, 0x80, 0x80, 0x86, 0x81, 0x41, 0x3F, 0x00
};
const uint8_t icon_data_active[] = { // Full data flowing
    0xFC, 0x02, 0x81, 0x61, 0x01, 0x81, 0x01, 0xF1,   0x01, 0xFD, 0x01, 0x01, 0x01, 0x02, 0xFC, 0x00,
    0x3F, 0x40, 0x80, 0x80, 0x80, 0x8F, 0x80, 0x83,   0x80, 0x9F, 0x80, 0x86, 0x81, 0x41, 0x3F, 0x00
};
const uint8_t icon_data_closed[] = { //Port not Available
    0xF8, 0x04, 0x02, 0x12, 0x22, 0x42, 0x82, 0x02,   0x82, 0x42, 0x22, 0x12, 0x02, 0x04, 0xF8, 0x00,
    0x3F, 0x40, 0x80, 0x90, 0x88, 0x84, 0x82, 0x81,   0x82, 0x84, 0x88, 0x90, 0x80, 0x40, 0x3F, 0x00
};
const uint8_t icon_data_loopback[] = { //Port in Loopback mode
    0xFC, 0x02, 0x01, 0x01, 0x11, 0x11, 0x11, 0x11,   0x11, 0x11, 0x21, 0xC1, 0x01, 0x02, 0xFC, 0x00,
    0x3F, 0x40, 0x80, 0x88, 0x9C, 0xAA, 0x88, 0x88,   0x88, 0x88, 0x84, 0x83, 0x80, 0x40, 0x3F, 0x00
};

/*----------------- LED Blink setup --------------------------------*/
/*
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
*/

// --- Modulino Knob Read 16-bit rotation value ---
int16_t knob_read_rotation(void) {
    uint8_t reg = 0x00;
    uint8_t buf[9];


// Scan 12c bus for devices (for debugging)
    for(uint8_t reg = 1; reg < 127; reg++) {

        // Select register
    
        i2c_write_blocking(QWIIC_PORT, MODULINO_KNOB_ADDR, &reg, 1, true);

        i2c_read_blocking(QWIIC_PORT, MODULINO_KNOB_ADDR, buf,8, false);
        
        printf("address 0x%02X Data 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", reg, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        
    }


    return (int16_t)((buf[0] << 8) | buf[1]);
}



/*----------------- Read FPGA register 8 bit --------------------------------*/
int read_register8(int address, int *data ){
    #if FPGA_INTERFACE_TYPE == 1
        return spi_read_register8(address, data);  
    #else
        return uart_read_register8(address, data);
    #endif
}
/*----------------- Read FPGA register 16 bit --------------------------------*/
int read_register16(int address, int *data ){
    #if FPGA_INTERFACE_TYPE == 1
        return spi_read_register16(address, data);  
    #else
        printf("Error: UART not supported\n");
        return 1;
    #endif
}

/*----------------- Write FPGA register 8 bit --------------------------------*/
int write_register8(int address, int data ){
    #if FPGA_INTERFACE_TYPE == 1
        return spi_write_register8(address, data);  
    #else
        return uart_write_register8(address, data);
    #endif
}


int spi_read_register8(int address, int *data){
    uint8_t spi_data = 0;
    uint8_t tx_data[2];

    // Prepare the data to be sent
    tx_data[1] = FPGA_CMD_READ_REG;  // Read register command
    tx_data[0] = address;

   // Select the SPI device by setting CS low
    gpio_put(PIN_CS, 0);

    // Read 1 byte from the SPI device
    spi_write_blocking(SPI_PORT, tx_data, 2);
    spi_read_blocking(SPI_PORT, 0, &spi_data, 1);

    // Deselect the SPI device by setting CS high
    busy_wait_us(1);
    gpio_put(PIN_CS, 1);
    *data = spi_data;
    return 0; //no error
}


int spi_write_register8(int address, int data) {
    uint8_t spi_command = FPGA_CMD_WRITE_REG;  // Write register command
    uint8_t tx_data[3];

    // Prepare the data to be sent
    tx_data[0] = address;
    tx_data[1] = FPGA_CMD_WRITE_REG;
    tx_data[2] = data;

    // Select the SPI device by setting CS low
    gpio_put(PIN_CS, 0);

    // Write the address and data to the SPI device
    spi_write_blocking(SPI_PORT, tx_data, 3);
 //   spi_write_blocking(SPI_PORT, &tx_data[1], 1);
 //   spi_write_blocking(SPI_PORT, &tx_data[2], 1);

    // Deselect the SPI device by setting CS high
    gpio_put(PIN_CS, 1);

    return 0; // No error checking performed
}

int spi_write_array(uint8_t *data, size_t length) {

    // Select the SPI device by setting CS low
    gpio_put(PIN_CS, 0);

    // Write the address and data to the SPI device
    spi_write_blocking(SPI_PORT, data, length);

    // Deselect the SPI device by setting CS high
    gpio_put(PIN_CS, 1);

    return 0; // No error checking performed
}


/*
int uart_read_register8(int address, int *data){
    uint8_t tx_data[2];
    tx_data[0] = address;
    tx_data[1] = FPGA_CMD_READ_REG; //read register command
    uint8_t rx_buffer;

    // drain rx fifo
    while(uart_is_readable(UART_ID)){
        uart_getc(UART_ID);
    }
    uart_putc_raw(UART_ID, tx_data[0]);
    uart_putc_raw(UART_ID, tx_data[1]);
 
    if(uart_is_readable_within_us(UART_ID, 1000)){
        rx_buffer=uart_getc(UART_ID);
        *data = rx_buffer;
        return 0;
    }else return 1;
}

int uart_write_register8(int address, int data){

    if(uart_is_writable(UART_ID)){
        uart_putc_raw(UART_ID, address);
        uart_putc_raw(UART_ID, FPGA_CMD_WRITE_REG);
        uart_putc_raw(UART_ID, data);
        return 0;
    } else return 1; //error could not write to UART
}
*/

/*----------------- SPI Read FPGA register 16 bit --------------------------------*/
int spi_read_register16(int address, int *data){
    uint8_t spi_data[2];
    uint8_t tx_data[2];
    int msb;

    spi_data[0] = 0;
    spi_data[1] = 0;
    // Prepare the data to be sent
    tx_data[1] = FPGA_CMD_READ_REG;  // Read register command
    tx_data[0] = address;

   // Select the SPI device by setting CS low
    gpio_put(PIN_CS, 0);

    // Read 1 byte from the SPI device
    spi_write_blocking(SPI_PORT, tx_data, 2);
    spi_read_blocking(SPI_PORT, 0, &spi_data[0], 1);
    spi_read_blocking(SPI_PORT, 0, &spi_data[1], 1);

    // Deselect the SPI device by setting CS high
    busy_wait_us(1);
    gpio_put(PIN_CS, 1);
    msb = spi_data[1]; // convert to int before shifting
    *data = (msb <<8) + spi_data[0];

    return 0; //no error
}

/*-----------------  FPGA configuration (Slave Serial) --------------------------------*/
int config_fpga(){
    uint8_t config_data;
    unsigned int i;
    bool done;

    //Note: The config guide does not require the Prog pin to be used in Slave Serial mode, however
    // this way we can reconfigure the FPGA without power cycling  (should we want to do this in the future). 
    gpio_set_oeover(PIN_FPGA_PROG, GPIO_OVERRIDE_NORMAL);
        gpio_put(PIN_FPGA_PROG, 0);  // hold FPGA in reset
    sleep_ms(1);
    gpio_put(PIN_FPGA_PROG, 1);  // release reset and start programming configuration data
    // Note: The config guide suggests waiting for INITN to go high here but we will just wait a fixed time
    sleep_ms(1);  // tinitl max = 55ns

    done = gpio_get(PIN_FPGA_DONE);
    if (done) {
        printf("Warning: Done pin did not reset\n");  // DONE should be low here
    } 

    //Note: Board B2401 does not have CSN/SN (pin R8) connected, so we cannot program the FPGA on that board
    //using Slave SPI mode.  However we can use Slave Serial mode using the SPI interface to clock in the data.
    //Infact Slave serial mode is much simpler so we will keep using it until we find a need for the extra features of Slave SPI mode.

    //To guarantee proper recognition of the synchronization word it is recommended that the synchronization 
    //word always be preceded by a minimum of 128 ‘1’ bits.
    for (i = 0; i < 128/8; i++) {
        config_data = 0xFF;
        spi_write_blocking(SPI_PORT, &config_data, 1);
    }

    for (i = header; i < fpga_config_len; i++) {
        config_data = fpga_config[i];
        spi_write_blocking(SPI_PORT, &config_data, 1);
    }

    sleep_ms(10);
    done = gpio_get(PIN_FPGA_DONE);
    if (done) {
        printf("FPGA Bitstream loaded successfully!\n");
    } else {
        --i;// to show last byte written
        printf("FPGA Bitstream load failed. Done pin still low. %u : %x\n", i, config_data);
    }
    gpio_set_oeover(PIN_FPGA_PROG, GPIO_OVERRIDE_LOW);  //High Z, Allow JTAG port to program FPGA if needed 
return 0;
}

void display(ssd1306_t *disp) {

    // The small display only draws on odd numbered y axis
 //   const char *words[]= {"MODE:", "Future use"};
    char buf_mode[8];

    ssd1306_clear(disp);

        if (config_data.mode == CONFIG_SD) {
            if(config_data.sd_num > 0 && config_data.sd_num < 100){
                 snprintf(buf_mode, sizeof(buf_mode), "SD%d", config_data.sd_num);
            } else if(config_data.sd_num > 99){ // remove "D" if more than 99 SDs to save space on display
                snprintf(buf_mode, sizeof(buf_mode), "S%d", config_data.sd_num);
            } else {
                 snprintf(buf_mode, sizeof(buf_mode), "SD?");
            }
        } else if (config_data.mode == CONFIG_MD) {
            snprintf(buf_mode, sizeof(buf_mode), "MD");
        } else {
            snprintf(buf_mode, sizeof(buf_mode), "ERR");
        }
        ssd1306_draw_string(disp, 50, 0 * 18, 1, status.display_message);
        ssd1306_draw_string(disp, 0 * 16, 0 * 9, 2, buf_mode);
        if (config_data.mode == CONFIG_SD) {
            if(status.link1) {
                draw_icon(disp, 4*16, 16, 16, 16, icon_data_active);
            } else if(status.port1_loopback){ // if loopback mode enabled, show loopback icon
                 draw_icon(disp, 4*16, 16, 16, 16, icon_data_loopback);
            } else {
                draw_icon(disp, 4*16, 16, 16, 16, icon_data_empty);
            }
            if(status.link2) {
                draw_icon(disp, 0*16, 16, 16, 16, icon_data_active);
            } else {
                draw_icon(disp, 0*16, 16, 16, 16, icon_data_empty);
            }
        } else if (config_data.mode == CONFIG_MD) {
            draw_icon(disp, 4, 16, 16, 16, icon_data_closed);
            if(status.link1){
                 draw_icon(disp, 4*16+4, 16, 16, 16, icon_data_active);
            } else {
                draw_icon(disp, 4*16+4, 16, 16, 16, icon_data_empty);
            }
        } else {
            draw_icon(disp, 4*16, 16, 16, 16, icon_data_empty);
        }
        ssd1306_show(disp);
        status.update_display = 0; // Clear the update flag

}

// Draw Icon
void draw_icon(ssd1306_t *disp, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *icon_data) {
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            int byte_index = (i + (j / 8) * width);
            int bit_index = j % 8;
            if (icon_data[byte_index] & (1 << bit_index)) {
                ssd1306_draw_pixel(disp, x + i, y + j);
            } else {
               // ssd1306_clear_pixel(disp, x + i, y + j);
            }
        }
    }
}

void set_display_message(const char *message) {
    if (message == NULL) {
        status.display_message[0] = '\0';
        return;
    }

    size_t msg_len = strlen(message);
    if (msg_len >= MAX_MESSAGE_LENGTH) {
        msg_len = MAX_MESSAGE_LENGTH - 1;
    }

    memcpy(status.display_message, message, msg_len);
    status.display_message[msg_len] = '\0';
    status.update_display = 1; // Set flag to indicate a new message has been received
}

/*----------------- main --------------------------------*/

// Define the global variable
int scroll = 0;

int main()
{
    int x, error;

    // Initialize the stdio library
    sleep_ms(2000);  // wait for Windows serial to connect
    stdio_init_all();

    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sleep_ms(2000);  // wait for serial to connect
    printf("\n\n");
    if (stdio_usb_connected()) {
         printf("USB serial available\n");
    };
    // uart0 default 115200 baud rate on GP0 (TX) and GP1 (RX)
    // uart0 is connected to J11 for serial console and also goes
    // to the FPGA where it can be routed to J15 TX:GPIO14, RX:GPIO15
    
    if (uart_is_writable(uart0)) {
        printf("UART available. Default Speed: 115200\n");
    } else {  // this should never happen as uart0 is always available...
        printf("No serial connection detected.\n");
    }

    // SPI initialisation. This example will use SPI at 1MHz.
    x=spi_init(SPI_PORT, 1000*1000);
    printf("SPI clock rate %d Hz\n", x);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

 // Set SPI format: 8 bits per transfer, CPOL=0, CPHA=0, MSB-first
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // Initialize CS as a normal GPIO (SIO) pin
    gpio_init(PIN_CS);

    // setup FPGA programming pins as normal GPIO (SIO) pins
    gpio_init(PIN_FPGA_DONE);
    gpio_init(PIN_FPGA_PROG);
    gpio_set_oeover(PIN_FPGA_PROG, GPIO_OVERRIDE_LOW);  //High Z, Allow JTAG to program FPGA if needed 
    gpio_set_dir(PIN_FPGA_PROG, GPIO_OUT);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    config_data.sd_num = 0; // Default SD number is 0 which means not connected (if in SD mode)

    // Read config data from flash (stores MD or SD mode)
    load_config(&config_data);
        printf("SPE ");
    if (config_data.mode == 0) { // SD mode
        printf("SECONDARY (SD%d)", config_data.sd_num);
    } else if (config_data.mode == 1) { // MD mode
        printf("MAIN (MD)");
    }
    printf(" Device Controller\n");


    // setup I2C
    i2c_init(i2c1, 100 * 1000); // Use 100khz I2C clock
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
    
    ssd1306_t disp;
    disp.external_vcc=false;

    printf("Display screen ");

    error = ssd1306_init(&disp, 128, 32, 0x3C, i2c1);  // small screen 128x32 pixel
    if(error < 0){// positive error value is number of bytes written (good thing)
        disp.active = false;
        gpio_put(LED_PIN, 0);
        printf("not found\n"); // SSD1306 initialization failed!
        sleep_ms(1000);  // turn off led for one second if no display found
        gpio_put(LED_PIN, 1);
    }  
    else {
        printf("found\n");
        disp.active = true;
    }
    set_display_message("EtherMUX.com");
    if (disp.active) display(&disp);


    // Configure FPGA
 //   printf("Dissabled Program FPGA...\n");
    config_fpga();  // Board must be set to Slave Serial mode for this to work
  
    x=0;
    read_register8(FPGA_VERSION, &x);
    if (x == FPGA_EXPECTED_VERSION) {
        printf("FPGA Version %02x OK\n", x);
    } else if (x == 0x00){
        printf("FPGA Version %02x failed\n", x);
    } else {
        printf("FPGA Version mismatch: %02x (expected %02x)\n", x, FPGA_EXPECTED_VERSION);
    }

    // Initialize QWIIC I2C port
 //   i2c_init(i2c0, 100 * 1000); // Use 100khz I2C clock
 //   gpio_set_function(QWIIC_SDA, GPIO_FUNC_I2C);
 //   gpio_set_function(QWIIC_SCL, GPIO_FUNC_I2C);
 //   gpio_pull_up(QWIIC_SDA);
 //   gpio_pull_up(QWIIC_SCL);
  //  printf("Modulino Knob test starting...\n");
  //  knob_read_rotation();

    char input[MAX_COMMAND_LENGTH] = "test string";
    int index = 0;
    int loop_counter = 0;
    int hop_number = 0;
    int prev_hop_number = 0;
    uint8_t link, comm;

    comm_buffer_index[0] = 0;
    comm_buffer_index[1] = 0;
    status.link1 = 0;
    status.link2 = 0;
    status.update_display = 0;
    status.link_change_pending = 0;


    // phy 2 provides clock to phy 1, so we reset it first to avoid link instability during reset
    reset_phy(2); // reset phy 2
    reset_phy(1); // reset phy 1
    
    print_prompt();
    while (true) { // Loop forever
  
        gpio_put(LED_PIN, 0);

        if(get_command(input, &index)){
            parse_command(input);
        }
        else if(scroll > 0){
            print_rotation_sensor();
        }
        gpio_put(LED_PIN, 1);
        if(comm_try_receive_line(1)){
            printf("Received: %s\n", comm_buffer[0]);
            set_display_message(comm_buffer[0]);
        }
        if(comm_try_receive_line(2)){
            printf("Received: %s\n", comm_buffer[1]);
            set_display_message(comm_buffer[1]);
        }
 
        // stagger processing of different tasks to avoid latency spikes. 
        // For example if we check the link status every loop it can cause a long delay
        // in processing received serial data which can cause buffer overflows and lost data.
        if(loop_counter++ > 10000){ // print prompt every 10K loops to show alive status
            if (disp.active && status.update_display) {
                display(&disp);
            }
            loop_counter = 0;
        }
        switch (loop_counter)
        {
        case 1:
            link= get_link_status(1);
            if(link != status.link1){
                printf("Port1 Link: %s\n", link ? "UP" : "DOWN");
                status.link1 = link;
                status.update_display = 1; // Set flag to update display on next loop
                status.link_change_pending = 1; // Set flag to indicate a link change has occurred and is pending reporting
            }
            break;
        case 1000:
            link = get_link_status(2);
            if(link != status.link2){
                printf("Port2 Link: %s\n", link ? "UP" : "DOWN");
                status.link2 = link;
                status.update_display = 1; // Set flag to update display on next loop
                status.link_change_pending = 1; // Set flag to indicate a link change has occurred and is pending reporting

            }
            break;
        case 2000:
            if(status.link_change_pending){
                if(config_data.mode == CONFIG_SD ){ // if in SD mode and link goes down, enable loopback mode
                    if(status.link2 == 1 && status.link1 == 0){ 
                        set_loopback_mode(1); 
                        printf("Port1 loopback: Enabled\n");
                    } else { 
                        set_loopback_mode(0);
                        printf("Port1 loopback: Disabled\n");
                    }
                    if(status.link2 == 1) {
                        enable_comm(1);
                    } else {
                        enable_comm(0);
                    }

                }else { // if in MD mode
                    if(status.link1 == 1) {
                        enable_comm(1);
                    } else {
                        enable_comm(0);
                    }
                    printf("Port1 Link is %s\n", status.link1 ? "UP" : "DOWN");
                }


                status.link_change_pending = 0; // Clear pending status after reporting
            }
        case 9000:
            hop_number = get_hop();
            if(hop_number != prev_hop_number){
                printf("SD HOP# %d\n", hop_number);
                config_data.sd_num = hop_number;
                prev_hop_number = hop_number;
                status.update_display = 1; // Set flag to update display on next loop
            }
            break;
        }
    }
}

int comm_try_receive_char(uint8_t *out_char, uint8_t com_port) {
    // check the fifo empty flag to make sure there is data to read
    int x,empty_flagb;
    uint8_t spi_data = 0;
    uint8_t tx_data[2] = {com_port,FPGA_CMD_READ_RX_FIFO};  // address is com_port (1 or 2), command is read rx fifo

    //Verifly a valid load in the fpaga by checking the version register before trying to read data from the FPGA.
    read_register8(FPGA_VERSION, &x);
    if (x != FPGA_EXPECTED_VERSION) {
        return 0; // FPGA not responding, return error
    }
    read_register8(FPGA_SPI_RD, &x);
    if(com_port == 1){ // if com_port is 1, we check bit 0 of FPGA_SPI_RD for fifo empty flag
        empty_flagb = x & 0x01;
    } else if(com_port == 2){ // if com_port is 2, we check bit 1 of FPGA_SPI_RD for fifo empty flag
        empty_flagb = (x & 0x02) >> 1;
    } else {
        return 0; // invalid com_port
    }
    if(empty_flagb == 1){ //bit 0 is fifo empty flag
        //read a character from the FPGA
        // Select the SPI device by setting CS low
        gpio_put(PIN_CS, 0);
        // Read 1 byte from the SPI device
        spi_write_blocking(SPI_PORT, tx_data, 2);
        spi_read_blocking(SPI_PORT, 0, &spi_data, 1);
        // Deselect the SPI device by setting CS high
        busy_wait_us(1);
        gpio_put(PIN_CS, 1);
        *out_char = spi_data;

        return 1; //character received
    } else {
        return 0; // no Character received
    } 
}

// Try to receive a line of text from the communication interface. Returns 1 if a line was received, 0 if no complete line is available.
// Com_port =1 is outbound MD to SD direction, com_port=2 is inbound SD to MD direction.  This allows us to have separate buffers and avoid conflicts if data is received in both directions at the same time.
int comm_try_receive_line(uint8_t com_port) {
    uint8_t received_char, overflow;
    
    overflow = 0;
    if(com_port < 1 || com_port > 2){
        printf("Error: Invalid com_port %d\n", com_port);
        return 0; // invalid com_port
    }

    while (comm_try_receive_char(&received_char, com_port)) {
        if (comm_buffer_index[com_port-1] < MAX_COMM_BUFFER_SIZE - 1) { // Leave space for null terminator
            comm_buffer[com_port-1][comm_buffer_index[com_port-1]] = received_char;
            comm_buffer_index[com_port-1] ++;
            if (received_char == '\r') { // Carriage return indicates end of line
                comm_buffer[com_port-1][comm_buffer_index[com_port-1]] = '\0'; // Null-terminate the string
                comm_buffer_index[com_port-1] = 0; // Reset buffer index for next line
                return 1; // Line received
            }
        } else {
            // Buffer overflow, reset index but keep draining the fifo
            overflow = 1;
        }
    }
    if(overflow){
        comm_buffer_index[com_port-1] = 0;
        printf("Error: RX Buffer overflow on com_port %d\n", com_port);
    }
    return 0; // No complete line received yet
}

int get_hop() {
    int x,error;
    if(config_data.mode == CONFIG_MD){
        return 0;
    } else{
        error = read_register8(FPGA_HOP, &x);
        return x;
    }
}