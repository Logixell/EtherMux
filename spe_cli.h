#ifndef spe_cli_h
#define spe_cli_h
    
    #define MAX_COMMAND_LENGTH 50
    #define MAX_ARGUMENTS 6
    // select only one of the following loopback options
    #define LOOPBACK_MODE NONE  //Valid Arguments: PCS or AFE or RMII or DIGITAL
    #define AFE 1
    #define PCS 2
    #define RMII 3
    #define DIGITAL 4
    #define NONE 5  //you must use None if you want the packet generator to send out to MDI

    // SMI registers
    #define REGCR 0x0D
    #define ADDAR 0x0E
    // FPGA Interface Type

    int parse_command(char *input);
    int process_command(int argc, char *argv[MAX_ARGUMENTS]);
    int reset_phy(int device);
    int phy_tdr(int device);
    int phy_sqi(int device);
    int read_smi_ext(int device, int address, int *data);
    int read_smi(int device, int address, int *data);
    int write_smi_ext(int device, int address, int data);
    int write_smi(int device, int address, int data);

//    int copy_argument(char *input, char *output, int arg);
    int get_command(char *input,int *pIndex);
    int print_rotation_sensor();
    int enable_data_generator_checker(int device);
    int send_packet();

#endif 