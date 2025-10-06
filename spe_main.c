#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
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

#define PIN_STEP 14 //GP14=pin 19
#define PIN_DIR  15 //GP15=pin 20

//FPGA config (also uses spi0 above)
#define PIN_FPGA_PROG  13 //GP13=pin 17
#define PIN_FPGA_DONE   12 //GP12=pin 16  --BUG not connected on v0.1 board

//Display
#define I2C_SDA 4 // GP4=pin 6
#define I2C_SCL 5 // GP5=pin 7


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4  //GP4=pin 6
#define UART_RX_PIN 5  //GP5=pin 7


#define MAX_TOKENS 10
#define MAX_TOKEN_LENGTH 50

#define MAX_COMM_BUFFER_SIZE 256

uint8_t comm_buffer[MAX_COMM_BUFFER_SIZE];
int comm_buffer_index = 0;

config_data_t config_data = {CONFIG_MAGIC, CONFIG_VERSION, 0, 0.0f, 0}; // Default mode (on a new chip) is SD


/*----------------- LED Blink setup --------------------------------*/
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
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

/*-----------------  FPGA configuration --------------------------------*/
int config_fpga(){
    gpio_put(PIN_FPGA_PROG, 0);  // hold FPGA in reset
    sleep_ms(1);
    gpio_put(PIN_FPGA_PROG, 1);  // release reset and start programming configuration data
    sleep_ms(1);
    //BUG Board B2401 does not have CSN/SN (pin R8) connected, so we cannot program the FPGA on that board
    gpio_put(PIN_CS, 0);  // start SPI transfer

    for (int i = 0; i < fpga_config_len; i++) {
        spi_write_blocking(SPI_PORT, &fpga_config[i], 1);
    }

    gpio_put(PIN_CS, 1);  // end SPI transfer

    sleep_ms(10);
    bool done = gpio_get(PIN_FPGA_DONE);
    if (done) {
        printf("Bitstream loaded successfully!\n");
    } else {
        printf("Bitstream load failed.\n");
    }


}

void display(void) {
    const char *words[]= {"OpenSPE.org", "MODE:", "   MD     ",
        "uvwxyz1234567890","OpenSPE.org", "MASTER DEVICE"};

    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c_default);
    ssd1306_clear(&disp);

    printf("DISPLAY!\n");

    char buf_mode[8];

        //sleep_ms(1000);
        if (config_data.mode == 0) {
            snprintf(buf_mode, sizeof(buf_mode), "SD");
        } else if (config_data.mode == 1) {
            snprintf(buf_mode, sizeof(buf_mode), "MD");
        } else {
            snprintf(buf_mode, sizeof(buf_mode), "ERR");
        }
        //ssd1306_draw_string(&disp, 0, 0*16, 2, words[0]);
        ssd1306_draw_string(&disp, 10 * 2, 1 * 16, 2, words[1]);
        ssd1306_draw_string(&disp, 10 * 8, 1 * 16, 2, buf_mode);
        ssd1306_show(&disp);

}

/*----------------- main --------------------------------*/
#include "spe_main.h"

// Define the global variable
int scroll = 0;

int main()
{
    int x;

    // Initialize the stdio library
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    x=spi_init(SPI_PORT, 1000*1000);
    printf("\nSPI clock rate %d Hz\n", x);
    printf("SPE FPGA Controller\n");

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
 
    gpio_set_function(PIN_STEP,  GPIO_FUNC_SIO);
    gpio_set_function(PIN_DIR,   GPIO_FUNC_SIO);
    gpio_set_dir(PIN_STEP, GPIO_OUT);
    gpio_set_dir(PIN_DIR, GPIO_OUT);
    //setup FPGA programming pins
    gpio_set_function(PIN_FPGA_DONE, GPIO_FUNC_SIO);
    gpio_set_function(PIN_FPGA_PROG, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_FPGA_PROG, GPIO_OUT);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded blink program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    uart_set_format(UART_ID, 8, 2, UART_PARITY_NONE);   // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    
    // Send out a string, with CR/LF conversions
    //uart_puts(UART_ID, " Hello, UART!\n");
    //int magicNumber = 0x1f;
    // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart

    // Read config data from flash
    load_config(&config_data);

    // setup I2C
    i2c_init(i2c_default, 100 * 1000); // Use i2c_default instead of i2c0
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    printf("jumping to display...\n");
    display();


    // Configure FPGA

    config_fpga();

    char input[MAX_COMMAND_LENGTH] = "test string";
    int index = 0;
    uint8_t comm;

    gpio_put(PIN_DIR, 0);
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
            printf("Received: %s", comm_buffer);
        }
        gpio_put(PIN_STEP, 0);
        busy_wait_ms(1);
        gpio_put(PIN_STEP, 1);
        busy_wait_ms(1);


    }
}

int comm_try_receive_char(uint8_t *out_char) {
    // check the fifo empty flag to make sure there is data to read
    int x,error;
    uint8_t spi_data = 0;
    uint8_t tx_data[2] = {0,FPGA_CMD_READ_RX_FIFO};  // address is don't care for fifo read

    error = read_register8(FPGA_SPI_RD, &x);
    if(x & 0x01){ //bit 0 is fifo empty flag
        return 0; // No character received
    } else {
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
        return 1; // Character received
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