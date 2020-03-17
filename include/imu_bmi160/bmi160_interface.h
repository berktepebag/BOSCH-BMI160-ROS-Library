#ifndef _BMI160_INTERFACE_H_
#define _BMI160_INTERFACE_H_

#include  <linux/i2c-dev.h>  // manage i2c interface in ARM
#include  <stdint.h>         //All these libraries are needed to 
#include  <iostream>         //use the functions we'll implement, 
#include  <time.h>           //such as delay
#include  <stdio.h>
#include  <stdint.h>
#include  <stdlib.h>
#include  <string.h>
#include  <math.h>
#include  <unistd.h>
#include  <sys/ioctl.h>
#include  <sys/types.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <unistd.h>

#include "bmi160.h"             //include the bmi160 repo headers
#include "bmi160_defs.h"        //

void delay_ms(uint32_t period);

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

class BMI160{
    public:
        BMI160();
        bmi160_dev initialize(int8_t &rslt);

        void set_sensor_settings(struct bmi160_dev *dev, int mode, int8_t &rslt);

        void read_sensor_data(struct bmi160_dev *dev, int8_t &rslt);
};


#endif