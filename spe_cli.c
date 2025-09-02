/* NO SDK LIBRARIES ARE ALLOWED IN THIS FILE 
    functions in this libary are not to include any SDK libraries.
    This file is to remain portable to any SDK.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spe_cli.h"
#include "spe_main.h"
#include "pico/stdlib.h"

#define HISTORY_SIZE 10 // Maximum number of commands to store in history

char command_history[HISTORY_SIZE][MAX_COMMAND_LENGTH]; // Command history buffer
int history_index = 0; // Index for the next command to be stored
int history_scroll = -1; // Index for scrolling through history (-1 means no scrolling)

/*----------------- parse command --------------------------------*/
int parse_command(char *input){
    char *argv[MAX_ARGUMENTS]; // Array to hold command line arguments
    int argc;

    //volatile int number_of_arguments = 0;
    //int arg = 0;
    //int error;
    //int address, data;
    //char string_out[MAX_COMMAND_LENGTH];
    //char character;

    int input_length = strlen(input);
    
    // Remove trailing newline character
    input[strcspn(input, "\n")] = '\0';

    // Tokenize the input into arguments
    argc = 0;
    char *token = strtok(input, " ");
    while (token != NULL && argc < MAX_ARGUMENTS) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }
    printf("\n>");

    // Process the command
    if (argc > 0) {
        process_command(argc, argv);
        printf(">");
    } else {
        scroll = 0;
    }
}

int process_command(int argc, char *argv[MAX_ARGUMENTS]) {
    int address, data, error, device;
    char *endptr;

    if ((strcmp(argv[0], "read") == 0) || 
       (strcmp(argv[0], "r") == 0)) {
        if(argc == 2){ // number of arguments
            address = strtol(argv[1], &endptr, 0);
            if (*endptr != '\0') {
                printf("Error: Invalid address '%s'\n", argv[1]);
                return 1;
            }
            error = read_register8(address, &data);
            if(!error){
                printf("fpga register 0x%02X:0x%02X (%d:%d)\n", address, data, address, data);
            }else{
                printf("Error: No Response\n");
            }
        }else if (argc ==3) {
            if(strcmp(argv[1], "smi1") == 0 || strcmp(argv[1], "smi2") == 0){
                if(strcmp(argv[1], "smi1") == 0){
                    device = 1;
                }else if(strcmp(argv[1], "smi2") == 0){
                    device = 2;
                }
                address = strtol(argv[2], &endptr, 0);
                if (*endptr != '\0') {
                    printf("Error: Invalid address '%s'\n", argv[1]);
                    return 1;
                }
                error = read_smi_ext(device, address, &data);
                if(!error){
                    printf("SMI%d register 0x%02X:0x%02X (%d:%d)\n", device, address, data, address, data);
                }else{
                    printf("Error: No Response\n");
                }
            }else{
                printf("Error: smi1 or smi2\n");
            }
        }else printf("Error: r [address]\n");
    }else if ((strcmp(argv[0], "write") == 0) ||
             (strcmp(argv[0], "w") == 0)){
        if (argc == 3){ // number of arguments
            address = strtol(argv[1], &endptr, 0);
            if (*endptr != '\0') {
                printf("Error: Invalid address '%s'\n", argv[1]);
                return 1;
            }
            data = strtol(argv[2], &endptr, 0);
            if (*endptr != '\0') {
                printf("Error: Invalid data'%s'\n", argv[2]);
                return 1;
            }

            error = write_register8(address, data);
            if (!error){
                printf("write fpga register 0x%02X:0x%02X (%d:%d)\n", address, data, address, data);
            } else {
                printf("Error: UART not ready\n");
            }
        }else if (argc == 4) { 
            if(strcmp(argv[1], "smi1") == 0 || strcmp(argv[1], "smi2") == 0){
                if(strcmp(argv[1], "smi1") == 0){
                    device = 1;
                }else if(strcmp(argv[1], "smi2") == 0){
                    device = 2;
                }   
                
                address = strtol(argv[2], &endptr, 0);
                if (*endptr != '\0') {
                    printf("Error: Invalid address '%s'\n", argv[2]);
                    return 1;
                }
                data = strtol(argv[3], &endptr, 0);
                if (*endptr != '\0') {
                    printf("Error: Invalid data'%s'\n", argv[3]);
                    return 1;
                }
                error = write_smi_ext(device, address, data);
                if(!error){
                    printf("SMI%d write register 0x%02X:0x%02X (%d:%d)\n", device, address, data, address, data);
                }else{
                    printf("Error: No Response\n");
                }
            }
        }else printf("Error: w [address] [data]\n");
    
    }else if (strcmp(argv[0], "rot") == 0) {
        if(argc == 1){ // number of arguments
            print_rotation_sensor();
            scroll = 1;
        }else printf("Error: invalid arguments t\n");

    }else if (strcmp(argv[0], "reset") == 0) {
        if(argc == 1){ // general board reset all
            device = 1;
            while(device < 3){ //reset ethernet phys
                error = reset_phy(device);
                if(!error){
                    printf("Reset PHY %d\n", device);
                }else{
                   printf("Error: No Response\n");
                }
                ++device;
            };

        }else if(argc == 3){ // number of arguments
            if(strcmp(argv[1], "phy") == 0){
                device = atoi(argv[2]);
                error = reset_phy(device);
                if(!error){
                    printf("Reset PHY %d\n", device);
                }else{
                    printf("Error: No Response\n");
                }
            } else {
                printf("Error: reset [phy] [device]\n");
            }
        }else printf("Error: reset [device]\n");

    }else if ((strcmp(argv[0], "phy1") == 0) || (strcmp(argv[0], "phy2") == 0)){
        if(strcmp(argv[0], "phy1") == 0){
            device = 1;
        }else if(strcmp(argv[0], "phy2") == 0){
            device = 2;
        } 
        if(strcmp(argv[1], "tdr") == 0){
            error = phy_tdr(device);
            if(!error){
                printf("TDR test complete\n");
            }else{
                printf("Error: No Response\n");
            }
        } else if(strcmp(argv[1], "sqi") == 0){
            error = phy_sqi(device);
            if(!error){
                printf("SQI test complete\n");
            }else{
                printf("Error: No Response\n");
            }
        } else if (strcmp(argv[1], "gen") == 0){
            error= enable_data_generator_checker(device);
            if(!error){
                printf("Generator test complete\n");
            }else{
                printf("Error: No Response\n");
            }
        } else if (strcmp(argv[1], "send") == 0){
            error= send_packet(device);
            if(!error){
                printf("TX packets enabled\n");
            }else{
                printf("Error: No Response\n");
            }


        }else printf("Error: tdr or sqi\n");

    }else if ((strcmp(argv[0], "help") == 0) || (strcmp(argv[0], "?") == 0)) {
        printf("Commands:\n");
        printf(" read  (or r) [address]         - Read FPGA register\n");
        printf(" write (or w)[address] [data]   - Write FPGA register\n");
        printf(" read  (or r) smi(1 or 2) [address]       - Read SMIx register \n");
        printf(" write (or w) smi(1 or 2) [address] [data] - Write SMIx register\n");
        printf(" rot                  - Print rotation sensor\n");
        printf(" reset phy [device]   - Reset PHY [device=1 or 2]\n");
        printf(" phyx [tdr]           - Time Domain Reflectometry test\n");
        printf(" phyx [sqi]           - Signal Quality Indicator test\n");
        printf(" phyx [gen]           - Packet Generator\n");
        printf(" help (or ?)         - Show this help message\n");
    }else {
        printf("Command not found\n");
    }

    return 0;
}

/*--- SPx Time Domain Reflectometry test for ethernet PHY ---*/
int phy_tdr(int device) {
    int error, data, tdr_status, tdr_result;
    int tdr_control = 0x1E;  // TDR control register address
    int tdr_result_reg = 0x310; // TDR result register address

    // Step 1: Start TDR by setting bit[15] in register 0x1E
    error = write_smi_ext(device, tdr_control, 0x8000); // Set bit[15] to '1' to start TDR
    if (error) {
        printf("Error: Unable to start TDR\n");
        return error;
    }

    // Step 2: Wait 100ms for TDR to complete
    busy_wait_ms(100);

    // Step 3: Read register 0x1E[1:0] to check TDR execution status
    error = read_smi_ext(device, tdr_control, &tdr_status);
    if (error) {
        printf("Error: Unable to read TDR status\n");
        return error;
    }

    if ((tdr_status & 0x3) != 0x2) { // Check if TDR executed successfully (0x1E[1:0] == 0b10)
        printf("Error: TDR execution failed or incomplete (status: 0x%02X)\n", tdr_status & 0x3);
        return 1;
    }

    // Step 4: Read TDR results from register 0x310
    error = read_smi_ext(device, tdr_result_reg, &tdr_result);
    if (error) {
        printf("Error: Unable to read TDR results\n");
        return error;
    }

    // Step 5: Interpret TDR results
    printf("TDR Results:\n");
    printf("  Half Wire Open Detected: %s\n", (tdr_result & 0x100) ? "Yes" : "No");
    printf("  Cable Fault Detected: %s\n", (tdr_result & 0x80) ? "Yes" : "No");
    if (tdr_result & 0x80) { // If a cable fault is detected
        printf("  Cable Fault Type: %s\n", (tdr_result & 0x40) ? "OPEN" : "SHORT");
        printf("  Fault Location: %d meters\n", tdr_result & 0x3F);
    } else {
        printf("  No cable fault detected.\n");
    }

    return 0; // Success
}

/*--- SPx Signal Quality Indicator test for ethernet PHY ---*/
int phy_sqi(int device) {
    int error, reg_value, sqi, worst_sqi;
    int dsp_reg_71 = 0x871; // SQI register address for DP83TC812-Q1

    // Step 1: Read the SQI value from the SQI register (0x312)
    error = read_smi_ext(device, dsp_reg_71, &reg_value);
    if (error) {
        printf("Error: Unable to read SQI value\n");
        return error;
    }
    sqi = (reg_value >> 1) & 0x7; // Extract the SQI value from the register
    worst_sqi = (reg_value >> 5) & 0x7; // Extract the worst SQI value from the register

    // Step 2: Interpret the SQI value
    printf("Signal Quality Indicator Results:\n");
    printf("  Current SQI Value: %d\n", sqi);
    printf("  Worst SQI Value:   %d\n", worst_sqi);

    // Step 3: Provide a qualitative assessment of the SQI value
    if (sqi >= 7) {
        printf("  Signal Quality:   Excellent\n");
    } else if (sqi >= 5) {
        printf("  Signal Quality:   Good\n");
    } else if (sqi >= 3) {
        printf("  Signal Quality:   Fair\n");
    } else {
        printf("  Signal Quality:   Poor\n");
    }

    return 0; // Success
}

/*----------- reset and initialize ethernet phy -------------------*/
int reset_phy(int device){
    int error;
    int data = 0x00;

    error = write_smi_ext(device, 0x1F, 0x8000); // Hardware reset PHY
//busy_wait_ms(1000); // wait for 1ms

    error = read_register8(FPGA_CTRL, &data); 
    if(device == 1 ){
        data = data & 0xFE; // clear reset bit
        error = write_register8(FPGA_CTRL, data); 
        data = data | 0x03; // set wake and reset bit (creates 37uS resetb pulse)
        error = write_register8(FPGA_CTRL, data); 
        error = write_smi_ext(device, 0x1834, 0xC000); // Bit 14:1b = Configure PHY1 as MASTER

    } else if (device == 2 ){
        data = data & 0xEF; // clear reset bit
        error = write_register8(FPGA_CTRL, data); 
        data = data | 0x30; // set wake and reset bit
        error = write_register8(FPGA_CTRL, data); 
        error = write_smi_ext(device, 0x1834, 0x8000); // Bit 14:0b = Configure PHY2 as Slave ie: turn off link

    }
    busy_wait_ms(1000); // wait for 1ms
    // Setup PHY in RMII Master mode 011
    
    //error = write_smi_ext(device,  0x001F, 0x8000); //Hardware reset
    //busy_wait_ms(1000); // wait for 1ms
    // read regiter 0x45D to check strap configuration
    // read register 0x1 bit 2 for link status
    //error = write_smi_ext(device, 0x0648, 0x0160); // Bug no effect observed RMII Contol 1 Register 
    //error = write_smi_ext(device, 0x064A, 0x0400); // Bug no effect observed RMII Override Contol 1 Register 
    error = write_smi_ext(device, 0x18B, 0x1C4B); // Bug no effect observed Bit 6: Autonomous mode
    //error = write_smi_ext(device, 0x453, 0x0081); // Bug no effect observed Clockout mux
//    error = write_smi_ext(device, 0x1F, 0x4000); // Software restart PHY

//Register setup from linux driver static const struct DP83TC812_init_reg DP83TC812_master_cs2_0_init[] = {
/*
error = write_smi_ext(device,  0x001F, 0x8000); //Hardware reset
error = write_smi_ext(device,  0x0523, 0x0001); //unlisted register Transmit disable
error = write_smi_ext(device,  0x1834, 0xC001);
error = write_smi_ext(device,  0x081C, 0x0FE2); //unlisted register
error = write_smi_ext(device,  0x0872, 0x0300); //unlisted register
error = write_smi_ext(device,  0x0879, 0x0F00); //unlisted register
error = write_smi_ext(device,  0x0806, 0x2952); //unlisted register
error = write_smi_ext(device,  0x0807, 0x3361); //unlisted register
error = write_smi_ext(device,  0x0808, 0x3D7B); //unlisted register
error = write_smi_ext(device,  0x083E, 0x045F); //unlisted register
error = write_smi_ext(device,  0x1834, 0xC001); // Master mode
error = write_smi_ext(device,  0x0862, 0x00E8); //unlisted register
error = write_smi_ext(device,  0x0896, 0x32CB); //unlisted register
error = write_smi_ext(device,  0x003E, 0x0009); //unlisted register
error = write_smi_ext(device,  0x001F, 0x4000); //Software reset
error = write_smi_ext(device,  0x0523, 0x0000); //unlisted register
*/
busy_wait_ms(1000); // wait for 1ms
//error = write_smi_ext(device,  0x01F, 0x4000); //Software reset




    return error;
}

/*---- Read Single Pair Ethernet Phy Serial Management inteface extended registers ----*/
int read_smi_ext(int device, int address, int *data){
    int error, result;

    if(address < 32){ // address is in the first 32 registers
        error = read_smi(device, address, data);
    }else if(address < 0x1000){ // MMD1F - Read Address Operation
        error = write_smi(device, REGCR, 0x1f);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x401F);
        error = read_smi(device, ADDAR, &result);
        *data = result; 
    }else if(address < 0x1FFF){ //MMD1 0x1000-0x1FFF - Read Address Operation
        address = address & 0xFFF; // convert to MMD1 address
        error = write_smi(device, REGCR, 0x1);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x4001);
        error = read_smi(device, ADDAR, &result);
        *data = result;
    }else if(address < 0x3002){ //MMD3 0x3000-0x3001 - Read Address Operation
        address = address & 0xFFF; // convert to MMD1 address
        error = write_smi(device, REGCR, 0x3);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x4003);
        error = read_smi(device, ADDAR, &result);
        *data = result;

    } else error =1; //error address not found

    return error;
}    
/*--------- Read Single Pair Ethernet Phy Serial Management Interface ----*/
int read_smi(int device, int address, int *data){
    int error;
    int data_lsb, data_msb;
    int smi_ctl_addr = 0x00; // control register address

    // check device address
    if(device == 1){
        smi_ctl_addr = FPGA_PHYA_ADDR; 
    }else if(device == 2){
        smi_ctl_addr = FPGA_PHYB_ADDR; 
    }else {
        return 1;
    } //error device not found
    smi_ctl_addr = smi_ctl_addr | 0x40; // control register address Bits [6 : 5]  01=1 write, 10=read,00=idle Bits [4:0]  Device address
   
    error = write_register8(FPGA_SMI_ADDR, address); // SMI register address
    error = write_register8(FPGA_SMI_CTL_ADDR, smi_ctl_addr); // write to SMI control register
    error = write_register8(FPGA_SMI_CTL_ADDR, 0); // clear SMI control register
    busy_wait_ms(1);

    error = read_register8(FPGA_SMI_DATA_LSB, &data_lsb); 
    error = read_register8(FPGA_SMI_DATA_MSB, &data_msb); 

    *data = (data_msb << 8) + data_lsb; // combine the two bytes into one word

    return 0;
}

/*---- write Single Pair Ethernet Phy Serial Management inteface extended registers ----*/
/* 
Steps to Access Clause 45 Registers
Write the MMD and Operation Type to REGCR (0xD):

Bits [4:0]: MMD (MDIO Manageable Device).
Bits [15:14]: Operation type:
00: Address operation.
01: Write operation.
10: Post-read increment address.
11: Read operation.
Write the Register Address to ADDAR (0xE):

Specify the target register address within the selected MMD.
Perform the Desired Operation:

For a write operation:
Write the data to ADDAR (0xE).
For a read operation:
Read the data from ADDAR (0xE).
*/
int write_smi_ext(int device, int address, int data){
    int error;

    if(address < 32){ // address is in the first 32 registers
        error = write_smi(device, address, data);
    }else if(address < 0x1000){ // MMD1F - Write Address Operation
        error = write_smi(device, REGCR, 0x1f);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x401F);
        error = write_smi(device, ADDAR, data);
    }else if(address < 0x1FFF){ //MMD1 0x1000-0x1FFF - Write Address Operation
        address = address & 0xFFF; // convert to MMD address
        error = write_smi(device, REGCR, 0x1);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x4001);
        error = write_smi(device, ADDAR, data);
    }else if(address < 0x3002){ //MMD3 0x3000-0x3001 - Write Address Operation
        address = address & 0xFFF; // convert to MMD address
        error = write_smi(device, REGCR, 0x3);
        error = write_smi(device, ADDAR, address);
        error = write_smi(device, REGCR, 0x4003);
        error = write_smi(device, ADDAR, data);

    } else error =1; //error address not found

    return error;
}    


/*--------- Write Single Pair Ethernet Phy Serial Management Interface ----*/
int write_smi(int device, int address, int data){
    int error;
    int data_lsb, data_msb;
    int smi_ctl_addr = 0x00; // control register address

    data_lsb = data & 0x00FF; // get LSB
    data_msb = (data >> 8) & 0x00FF; // get MSB

    // check device address
    if(device == 1){
        smi_ctl_addr = FPGA_PHYA_ADDR; 
    }else if(device == 2){
        smi_ctl_addr = FPGA_PHYB_ADDR; 
    }else {
        return 1;
    } //error device not found
    smi_ctl_addr = smi_ctl_addr | 0x20; // control register address Bits [6 : 5]  01=1 write, 10=read,00=idle Bits [4:0]  Device address
 
    error = write_register8(FPGA_SMI_ADDR, address); // SMI register address
    error = write_register8(FPGA_SMI_DATA_LSB, data_lsb); 
    error = write_register8(FPGA_SMI_DATA_MSB, data_msb); 
    error = write_register8(FPGA_SMI_CTL_ADDR, smi_ctl_addr); // write to SMI control register
    error = write_register8(FPGA_SMI_CTL_ADDR, 0); // clear SMI control register

    return 0;
}

/*----------------- print roTation sensor --------------------------------*/
int print_rotation_sensor(){
    int error, rotations, position;


    error = read_register16(FPGA_SENT_ROT_LSB, &rotations);
    error = read_register16(FPGA_SENT_LSB, &position);

    if(!error){
        //printf("\033[2J\033[1;1H");//clear screen and move cursor to home
         
        printf("\033[?25l");// Turn off the cursor
 //       printf("\033[32m");// set color green
 //       printf("\033[34m");// set color blue (unreadable)
 //       printf("\033[36m");// set color cyan
        printf("\033[33m ");//set color yellow
    
        if(scroll == 0){printf("\033[2J");}//clear screen if it is the first time around
        printf("\033[1;1H");//move cursor to home
        printf("Rotations: 0x%02X (%d)      \n", rotations, rotations);
        printf("\033[36m");// set color cyan
        printf("Position:  0x%02X (%d)      \n", position, position);

        printf("\033[0m");// reset color
        printf("\033[?25h"); // Turn on the cursor
    }else{
        printf("Error: UART not ready\n");
    }
    return 0;
}

/*----------------- get CLI input --------------------------------*/
int get_command(char *input, int *pIndex) {  // This is a non-blocking function
    int c = stdio_getchar_timeout_us(0);  
    if (c != PICO_ERROR_TIMEOUT) {
        if (c == 10 || c == 13) { // Line feed or carriage return
            input[*pIndex] = '\000';
            *pIndex = 0;

            // Save the command to history if it's not empty
            if (strlen(input) > 0) {
                strncpy(command_history[history_index], input, MAX_COMMAND_LENGTH);
                history_index = (history_index + 1) % HISTORY_SIZE; // Circular buffer
            }
            history_scroll = -1; // Reset history scroll
            return 1;
        }
        if (c == 8 || c == 127) { // Backspace or delete
            if (*pIndex > 0) { // Prevent from going below zero
                *pIndex = *pIndex - 1;
                input[*pIndex] = '\000';
                if (CLI_ECHO) { stdio_putchar(c); }
            }
        }
        if (c == 27) { // Escape character
            c = stdio_getchar_timeout_us(1000);  // Read next char
            if (c == '[') { // Control sequence introducer
                c = stdio_getchar_timeout_us(1000);  // Read next char
                if (c == 'A') { // Up arrow
                    if (history_scroll < HISTORY_SIZE - 1 && history_scroll < history_index) {
                        history_scroll++;
                        int history_pos = (history_index - 1 - history_scroll + HISTORY_SIZE) % HISTORY_SIZE;
                        strncpy(input, command_history[history_pos], MAX_COMMAND_LENGTH);
                        *pIndex = strlen(input);
                        printf("\033[2K"); // Clear the current line
                        printf("\r>%s", input); // Display the command
                    }
                }
                if (c == 'B') { // Down arrow
                    if (history_scroll > 0) {
                        history_scroll--;
                        int history_pos = (history_index - 1 - history_scroll + HISTORY_SIZE) % HISTORY_SIZE;
                        strncpy(input, command_history[history_pos], MAX_COMMAND_LENGTH);
                        *pIndex = strlen(input);
                        printf("\033[2K"); // Clear the current line
                        printf("\r>%s", input); // Display the command
                    } else if (history_scroll == 0) {
                        history_scroll = -1;
                        *pIndex = 0;
                        input[0] = '\0';
                        printf("\033[2K"); // Clear the current line
                        printf("\r>"); // Clear the input
                    }
                }
            }
        } else if (*pIndex < MAX_COMMAND_LENGTH - 1) {
            if (c >= 32 && c <= 126) { // Valid characters
                input[*pIndex] = c;
                *pIndex = *pIndex + 1;
                if (CLI_ECHO) { stdio_putchar(c); }
            }
        }
    }
    return 0;
}

/*--- Enable Data Generator and Checker for ethernet PHY ---*/
int enable_data_generator_checker(int device) {
    int error;
    int reg_value, value, timeout;

    // Setup Data Generator/ checker modes
    //8.6.2.73 PRBS_CTRL_2 Register (Address = 61Ah) [Reset = 05DCh]
    error = write_smi_ext(device, 0x061A, 0x0020); // Frame length: 0x0020=32 bytes
    if(error) return 1;

    //8.6.2.74 PRBS_CTRL_3 Register (Address = 61Bh) [Reset = 007Dh] 125 bytes
    error = write_smi_ext(device, 0x061B, 0x0008); // Inter packet gap: 0x007D=125 bytes by default.  Testing revealed that 2 bytes will cause packet loss (phy rxdv goes high but data remains at 00 during the packet) 3 and above were ok.

    //8.6.2.81 PRBS_CTRL_4 Register (Address = 624h) [Reset = 5511h]
    // bits[15-8] Fixed data to be sent in Fix data mode
    int fixed_data = 0x03; // Fixed data (0-255)
    // bits[7-6] 0: Incremental data, 1: Fixed data, 2: PRBS, 3: PRBS
    int data_mode = 0; 
    // bits[5-3] Number of bytes of valid pattern in packet (Max - 6) default
    int valid_bytes = 1; // Number of valid bytes * 8 (0-7)
    // bits[2-0] Number of Configures the number of MAC packets to be transmitted by packet
    //generator, 0: 1 packet, 1: 10 packets, 2: 100 packets, 3: 1000 packets... 7: continuous
    int number_of_packets = 1; // Number of packets to be sent (1-7)

    reg_value = (fixed_data & 0xFF) << 8;  // bits[15-8]: Fixed data (8 bits)
    reg_value |= (data_mode & 0x3) << 6;    // bits[7-6]: Data mode (2 bits)
    reg_value |= (valid_bytes & 0x7) << 3;  // bits[5-3]: Number of valid bytes (3 bits)
    reg_value |= (number_of_packets & 0x7); // bits[2-0]: Number of packets (3 bits)
    error = write_smi_ext(device, 0x0624, reg_value); 


    // Figure 8-3. PCS Loopback with data generator
    // 8.6.2.10 BISCR Register (Address = 16h) [Reset = 0100h]
    if (LOOPBACK_MODE == PCS)
        error = write_smi_ext(device, 0x0016, 0x0102); // Enable PCS loopback
    else if (LOOPBACK_MODE == DIGITAL)
        error = write_smi_ext(device, 0x0016, 0x0104); // Enable Digital loopback 
    else if (LOOPBACK_MODE == AFE)
        error = write_smi_ext(device, 0x0016, 0x0108); // Enable AFE loopback (analog loopback)
    else if (LOOPBACK_MODE == RMII)
        error = write_smi_ext(device, 0x0016, 0x0150); //0x150 Enable MII (reverse) loopback
    else if (LOOPBACK_MODE == NONE)
        error = write_smi_ext(device, 0x0016, 0x0100); // back to default
    else{
        printf("Error: No valid loopback mode defined\n");
        return 1; // Return error if no valid loopback mode is defined
    }

    //Enable data generator/checker for MAC side
    // 8.6.2.72 PRBS_CTRL_1 Register (Address = 619h) [Reset = 0574h]
    if (LOOPBACK_MODE == RMII)
        error = write_smi_ext(device, 0x0619, 0x1224); // 0x1225 RMII loopback mode
    else 
        error = write_smi_ext(device, 0x0619, 0x1555); //Generates a packet

    //8.6.2.81 PRBS_CTRL_4 Register (Address = 624h) [Reset = 5511h]
    // this has been set above error = write_smi_ext(device, 0x0624, 0x55BF);


    // Check data for Cable side
    // 8.6.2.78 PRBS_STATUS_5 Register (Address = 620h) [Reset = 0000h]
    timeout = 0;
    do{
        error = read_smi_ext(device, 0x0620, &reg_value); 
        timeout ++;
    }while (((reg_value & 0x1000) == 0) && (timeout < 1000)); // Wait for packet generator done
    
    
    printf(" Packet Generator:     %sdone\n", (reg_value & 0x1000) ? "" : "Not ");
    printf("  Packet generator:     %sbusy\n", (reg_value & 0x800) ? "" : "Not ");
    printf("  Timeout Value:        %d\n",timeout);
    printf("  Packet cntr overflow: %s\n", (reg_value & 0x400) ? "Yes" : "No");
    printf("  Byte cntr overflow:   %s\n", (reg_value & 0x200) ? "Yes" : "No");
    printf("  PRBS sync loss:       %s\n", (reg_value & 0x200) ? "Had been lost" : "Never lost");
    printf("  PRBS error count:     %d\n", (reg_value & 0xFF));
    //8.6.2.91 RX_PKT_CNT_1 Register (Address = 63Ch) [Reset = 0000h]
    error = read_smi_ext(device, 0x063C, &reg_value); // Read Checker error value
    value = reg_value; // read only least significant word
    error = read_smi_ext(device, 0x063D, &reg_value); 
    error = read_smi_ext(device, 0x063E, &reg_value); // Read to clear registers
    printf("  Receive packet count: %d\n", value);
   
    // Disable loopback
    // 8.6.2.10 BISCR Register (Address = 16h) [Reset = 0100h]
    error = write_smi_ext(device, 0x0016, 0x0100); // back to default


    return error; // Success
}

/*----------------- Configure transmitt packet --------------------------------*/
int send_packet(int device) { 

    uint8_t data[47] = {0x00, // Address byte
                        0x03, // Command byte Write TX DPRAM
                        0x00,  // Packet data
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                        0x01,
                        0x00,
                        0x00,
                        0x01,
                        0x02,
                        0x03,
                        0x04,
                        0x05,
                        0x06,
                        0x07,
                        0x08,
                        0x09,
                        0x0A,
                        0x0B,
                        0x0C,
                        0x0D,
                        0x0E,
                        0x0F,
                        0x10,
                        0x11,
                        0x12,
                        0x13,
                        0x14,
                        0x15,
                        0x16,
                        0x17,
                        0x18,
                        0x19,
                        0x1A,
                        0x1B,
                        0x1C,
                        0x1D,
                        0x1E,
                        0x1F

    };  // 255 is max spi limit.

    // Send the packet data
    int error = spi_write_array((uint8_t *)data, sizeof(data));
    if (error) {
        printf("Error writing to TXDPRAM\n");
        return error;
    }

    // Enable packet transmission

    error = write_register8(0x19, 0x10); // enable packets on port 2
    if (error) {
        printf("Error enabling packet transmission: %d\n", error);
        return error;
    }

    return 0; // Success
}
