#include "imu_bmi160/bmi160_interface.h"

int file;
int adapter_nr = 2;
char filename[40];

BMI160::BMI160(){  //the main sensor interface class constructor

    sprintf(filename, "/dev/i2c-1");

    file = open(filename, O_RDWR);

    if ( file < 0)
    {
        std::cout << ("Failed to open the bus.") << std::endl;
        exit(1);
    }

    /* The I2C address */

    if (ioctl(file, I2C_SLAVE, BMI160_I2C_ADDR) < 0)
    {
        std::cout << ("Failed to acquire bus access and/or talk to slave.") << std::endl;
        exit(1);
    }
        else{
        printf("BMI160 found at 0x%02X\n",  BMI160_I2C_ADDR);
    }

}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
            // std::cout << ("i2c_read.") << std::endl;

            uint8_t buffer_write[2];
            memset(buffer_write, '\0',2);
            int n_writ;
            n_writ = 0;
            // Request data
            buffer_write[0] = reg_addr;

            n_writ = write(file,buffer_write,1);

            if(n_writ!=1){
                std::cout << ("BMI160 Reading Error (cannot request data): Failed to write.") << std::endl;
                return -1;
            }

            int n_read;
            // Read data
            n_read = read(file,data,len);
            if(n_read != len)
            {
                /* ERROR HANDLING: i2c transaction failed */
                std::cout << "BMI160 Reading Error (not enough data readed) :Failed to read" << std::endl;

                return -1;
            }

            return BMI160_OK;
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
            // std::cout << ("i2c_write.") << std::endl;
            
            uint8_t buffer_write[len+1]; // cast needed???!?
            memset(buffer_write,'\0',len+1);
            int n_writ;
            buffer_write[0] = reg_addr;

            for(int i = 0; i<len; i++)
            {
                buffer_write[i+1] = data[i];
            }
            n_writ = write(file,buffer_write, len+1);

            if( n_writ < len+1)
            {
                /* ERROR HANDLING: i2c transaction failed */
                std::cout << "BMI160 Writing Error (not enough data readed) :Failed to read" << std::endl;
                return -1;
            }
            return BMI160_OK;

}

void delay_ms(uint32_t period){
            // std::cout << ("i2c_delayed for: ") << period*1000 << " ms" << std::endl;
            usleep(period*1000);

}

bmi160_dev BMI160::initialize(int8_t &rslt){
    struct bmi160_dev dev;

    dev.id = BMI160_I2C_ADDR;
    dev.interface = BMI160_I2C_INTF;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_ms = delay_ms;

    rslt = bmi160_init(&dev);

    return dev;
}

