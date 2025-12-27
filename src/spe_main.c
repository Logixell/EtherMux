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
#define I2C0_SDA 4 // GP4=pin 6
#define I2C0_SCL 5 // GP5=pin 7
// I2C1 pins (Display)
#define I2C1_SDA 6 // GP6 = pin 
#define I2C1_SCL 7 // GP7 = pin 

#define MAX_TOKENS 10
#define MAX_TOKEN_LENGTH 50

#define MAX_COMM_BUFFER_SIZE 256

uint8_t comm_buffer[MAX_COMM_BUFFER_SIZE];
int comm_buffer_index = 0;

config_data_t config_data = {CONFIG_MAGIC, CONFIG_VERSION, 0, 0.0f, 0}; // Default mode (on a new chip) is SD


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

    //Note: The config guide does not require the Prog pin to be used in Slave Serial mode, however
    // this way we can reconfigure the FPGA without power cycling  (should we want to do this in the future). 
    gpio_put(PIN_FPGA_PROG, 0);  // hold FPGA in reset
    sleep_ms(1);
    gpio_put(PIN_FPGA_PROG, 1);  // release reset and start programming configuration data
    // Note: The config guide suggests waiting for INITN to go high here but we will just wait a fixed time
    sleep_ms(1);  // tinitl max = 55ns
    //BUG Board B2401 does not have CSN/SN (pin R8) connected, so we cannot program the FPGA on that board
    //using Slave SPI mode.  However we can use Slave Serial mode using the SPI interface to clock in the data.
    gpio_put(PIN_CS, 0);  // start SPI transfer


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

    gpio_put(PIN_CS, 1);  // end SPI transfer

    sleep_ms(10);
    bool done = gpio_get(PIN_FPGA_DONE);
    if (done) {
        printf("Bitstream loaded successfully!\n");
    } else {
        --i;// to show last byte written
        printf("Bitstream load failed. %u : %x\n", i, config_data);
    }


}

void display(ssd1306_t *disp, char *line1) {

    // The small display only draws on odd numberd y axis
    const char *words[]= {"MODE:", "Future use"};
    char buf_mode[8];

    ssd1306_clear(disp);

        if (config_data.mode == 0) {
            snprintf(buf_mode, sizeof(buf_mode), "SD");
        } else if (config_data.mode == 1) {
            snprintf(buf_mode, sizeof(buf_mode), "MD");
        } else {
            snprintf(buf_mode, sizeof(buf_mode), "ERR");
        }
        ssd1306_draw_string(disp, 50, 0 * 18, 1, line1);
       // ssd1306_draw_string(disp, 0 * 2, 1 * 9, 2, words[0]);
        ssd1306_draw_string(disp, 0 * 16, 0 * 9, 2, buf_mode);
       // ssd1306_draw_string(disp, 0 * 8, 3 * 16, 2, words[3]);
        //ssd1306_draw_line(disp, 0, 31, 60, 31);
        //ssd1306_draw_line(disp, 68, 31, 127, 31);
        //draw_icon(disp, 25, 16, 16, 16, icon_data_empty);
        if (config_data.mode == 0) {
            draw_icon(disp, 2*16, 16, 16, 16, icon_data_empty);
            draw_icon(disp, 5*16, 16, 16, 16, icon_data_active);
        } else if (config_data.mode == 1) {
            draw_icon(disp, 2*16, 16, 16, 16, icon_data_closed);
            draw_icon(disp, 5*16, 16, 16, 16, icon_data_active);
        } else {
            draw_icon(disp, 25, 16, 16, 16, icon_data_empty);
        }
        //draw_icon(disp, 85, 16, 16, 16, icon_data_active);
        ssd1306_show(disp);

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

/*----------------- main --------------------------------*/
#include "spe_main.h"

// Define the global variable
int scroll = 0;

int main()
{
    int x, error;

    // Initialize the stdio library

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
    gpio_set_dir(PIN_FPGA_PROG, GPIO_OUT);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);


    // Read config data from flash (stores MD or SD mode)
    load_config(&config_data);
        printf("SPE ");
    if (config_data.mode == 0) { // SD mode
        printf("SECONDARY");
    } else if (config_data.mode == 1) { // MD mode
        printf("MAIN");
    }
    printf(" Device Controller\n");

    // Configure FPGA
    printf("Program FPGA...\n");
    //config_fpga();  // Board must be set to Slave Serial mode for this to work
    // Wait for FPGA to be ready
    x=0;
    //do{
    //    read_register8(FPGA_VERSION, &x);
    //    sleep_ms(100);
    //}while (x != 0xa1);
    // Initialize FPGA registers
    if (config_data.mode == 0) { // SD mode
        write_register8(FPGA_CTRL, 0x01); // enable RX
    } else if (config_data.mode == 1) { // MD mode
        write_register8(FPGA_CTRL, 0x02); // enable TX
    }
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
    if (disp.active) display(&disp, "EtherMUX.com");



    char input[MAX_COMMAND_LENGTH] = "test string";
    int index = 0;
    uint8_t comm;

    reset_phy(1); // reset phy 1
    reset_phy(2); // reset phy 2
    
    printf(">");
    while (true) { // Loop forever

        if(get_command(input, &index)){
            parse_command(input);
        }
        else if(scroll > 0){
            print_rotation_sensor();
        }
        if(comm_try_receive_line(&comm_buffer[0], MAX_COMM_BUFFER_SIZE)){
            printf("Received: %s\r", comm_buffer);
            if (disp.active) display(&disp, comm_buffer);

        }

    }
}

int comm_try_receive_char(uint8_t *out_char) {
    // check the fifo empty flag to make sure there is data to read
    int x,error;
    uint8_t spi_data = 0;
    uint8_t tx_data[2] = {0,FPGA_CMD_READ_RX_FIFO};  // address is don't care for fifo read

    error = read_register8(FPGA_SPI_RD, &x);
    if(x & 0x01){ //bit 0 is fifo empty flag
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

int comm_try_receive_line(char *out_str, size_t max_length) {
    uint8_t received_char;

    while (comm_try_receive_char(&received_char)) {
        if (comm_buffer_index < MAX_COMM_BUFFER_SIZE - 1) { // Leave space for null terminator
            comm_buffer[comm_buffer_index] = received_char;
            comm_buffer_index ++;
            if (received_char == '\r') { // Carriage return indicates end of line
                comm_buffer[comm_buffer_index] = '\0'; // Null-terminate the string
                strncpy(out_str, (char *)comm_buffer, max_length);
                comm_buffer_index = 0; // Reset buffer index for next line
                return 1; // Line received
            }
        } else {
            // Buffer overflow, reset index
            comm_buffer_index = 0;
            return -1; // Indicate error
        }
    }
    return 0; // No complete line received yet
}
