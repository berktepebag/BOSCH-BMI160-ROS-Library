bmi160_acc_example#include "bmi160_interface.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <iomanip> // for using std::fixed and std::setprecision
#include <ctime>

bool SELF_TEST = false;
bool CALIBRATE_SENSOR_FOR_GRAVITY = true;

BMI160 bmi = BMI160();

void set_default_sensor_values(bmi160_dev *sensor)
{
    /* Select the Output data rate, range of accelerometer sensor */
    sensor->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ; // Min/Max Range -2000->2000 mili-G in 16 bits =>  2^16 = 65,535.  
    sensor->accel_cfg.range = BMI160_ACCEL_RANGE_2G; // Acceleration values between -2G/2G
    sensor->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    sensor->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensor->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 
}

void calibrate_sensor_for_gravity(bmi160_dev &sensor, int8_t &rslt,std::vector<double> &calibration_values)
{
    std::cout << "Calibrating sensor for gravitational forces" << std::endl;
    uint16_t n_samples = 1024;

    // double calibration_values[3];

    struct bmi160_sensor_data acc_data;

    double acc_x, acc_y, acc_z;

    for(int i=0; i<n_samples; i++)
    {
        rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &acc_data, NULL, &sensor);
        acc_x += acc_data.x;
        acc_y += acc_data.y;
        acc_z += acc_data.z;
    }

    calibration_values.push_back(acc_x/n_samples);
    calibration_values.push_back(acc_y/n_samples);
    calibration_values.push_back(acc_z/n_samples);


    std::cout << "Calibration completed! Saving to calibration.csv" << std::endl;

    // Saving to csv
    std::ofstream csv_file;
    csv_file.open("calibration.csv");

    for(std::vector<double>::iterator it = calibration_values.begin(); it!=calibration_values.end();it++)
    {
        std::cout << *it << std::endl;        
        csv_file << *it << "\n";
    }

    std::cout << "Saving to calibration.csv completed.." << std::endl;
}

double rawTOmilliG(double raw_data)
{
    double multiplier = 4000 / std::pow(2.0,16); 
    return raw_data*multiplier;
}

double milligTOms2(double milliG_data)
{
    return  milliG_data / 1000 * 9.81;
}

int main(){

    int8_t rslt = BMI160_OK;
    std::cout << "First: " << BMI160_OK << std::endl;

    bmi160_dev sensor;

    sensor = bmi.initialize(rslt);
    std::cout << "Second: " << (rslt!=BMI160_OK) << std::endl;

    // Set default sensor values
    set_default_sensor_values(&sensor);
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);

    if(SELF_TEST){
        rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);
        /* Utilize the enum BMI160_GYRO_ONLY instead of BMI160_ACCEL_ONLY
        to perform self test for gyro */
        if (rslt == BMI160_OK) {
            std::cout << "\n raw_acc SELF TEST RESULT SUCCESS" << std::endl;
        } 
        else {
            std::cout << "\n raw_acc SELF TEST RESULT FAIL" << std::endl;
        }
    }

    //Calibrate for gravitational forces or read from CSV file.
    std::vector<double> calibration_values = {};
    if(CALIBRATE_SENSOR_FOR_GRAVITY){
        calibrate_sensor_for_gravity(sensor, rslt, calibration_values);
    }
    else{
        calibration_values.clear();

        std::fstream csv_file;
        csv_file.open("calibration.csv");        

        std::string line;
        double cal_value;

        while(getline(csv_file, line)){
            double value = atof(line.c_str());
            calibration_values.push_back(value);
        }    

    }

//    for(std::vector<double>::iterator it = calibration_values.begin(); it!=calibration_values.end();++it)
//     {
//         std::cout << *it << std::endl;       
       
//     }

    
    struct bmi160_sensor_data raw_acc;

    // for (int i=0; i<500; i++){
    //     /* To read only raw_acc data */
    //     rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &raw_acc, NULL, &sensor);
    //     std::cout << "i: " << i << " acc x: " << raw_acc.x - calibration_values[0] << " acc y: " << raw_acc.y - calibration_values[1] << " acc z: " << raw_acc.z - calibration_values[2]  << std::endl;

    //     // std::cout << "i: " << i << " " << (rslt != BMI160_OK) << std::endl;
    //     sensor.delay_ms(50);
    // }

    double avg_x, avg_y, avg_z;
    uint8_t n_noise_samples = 30;

    std::vector<double> mechanical_filters = {150,150,350};
    
    time_t current_time, prev_time;
    bool first_run = true;
    double prev_acc_x, prev_acc_y, prev_acc_z, prev_vel_x, prev_vel_y, prev_vel_z = 0;
    
    int8_t zero_counter;

    while(true)
    {
        // Set current time and if it is first run set prev_time equal to the current time.
        time(&current_time);
        if(first_run)
        {
            std::cout << "Running for the first time..." << std::endl;
            prev_time = current_time;
            first_run = false;
        }
        
        //Low pass filter
        std::vector<double> average_readings = {};

        for (int i=0; i<n_noise_samples; i++){
            /* To read only raw_acc data */
            rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &raw_acc, NULL, &sensor);
            // std::cout << "i: " << i << " acc x: " << raw_acc.x - calibration_values[0] << " acc y: " << raw_acc.y - calibration_values[1] << " acc z: " << raw_acc.z - calibration_values[2]  << std::endl;

            avg_x += (raw_acc.x - calibration_values[0]);
            avg_y += (raw_acc.y - calibration_values[1]);
            avg_z += (raw_acc.z - calibration_values[2]);
            // std::cout << "i: " << i << " " << (rslt != BMI160_OK) << std::endl;
            sensor.delay_ms(1);
        }

        avg_x /= n_noise_samples;
        avg_y /= n_noise_samples;
        avg_z /= n_noise_samples;

        average_readings.push_back(avg_x);
        average_readings.push_back(avg_y);
        average_readings.push_back(avg_z);

        //Mechanical Filtering Window
        for(int i=0; i<3; i++)
        {
            if(fabs(average_readings[i]) < mechanical_filters[i])
            {
                average_readings[i] = 0;
            }
        }

        // Convert raw to milli-G and milli-G to m/s^2
        double acc_x = milligTOms2(rawTOmilliG(average_readings[0]));
        double acc_y = milligTOms2(rawTOmilliG(average_readings[1]));
        double acc_z = milligTOms2(rawTOmilliG(average_readings[2]));
        
        // std::cout << std::fixed << std::setprecision(2) << "avg x: " <<  acc_x << " m/s^2, avg_y: " << acc_y << " m/s^2 avg_z: " << acc_z << " m/s^2" << std::endl;

        // Time between two readings
        double delta_t_ = current_time - prev_time;
        // Calculate velocity according to the acceleration
        double vel_x = prev_vel_x + (prev_acc_x  + ( (acc_x-prev_acc_x) / 2) ) * delta_t_;
        double vel_y = prev_vel_y + (prev_acc_y  + ( (acc_y-prev_acc_y) / 2) ) * delta_t_;
        double vel_z = prev_vel_z + (prev_acc_z  + ( (acc_z-prev_acc_z) / 2) ) * delta_t_;

        std::cout << std::fixed << std::setprecision(2) << "vel x: " <<  acc_x << " m/s, vel_y: " << vel_y << " m/s vel_z: " << vel_z << " m/s" << std::endl;

        // std::cout << std::fixed << std::setprecision(2) << "vel x: " <<  acc_x << std::endl;

        //If we received 25 "0 m/s^2", it means we are not moving and set velocity to zero.
        if(acc_x == 0.0)
        {
            zero_counter++;
            if(zero_counter==25)
            {
                vel_x = 0;
                zero_counter = 0;
            }
        }

        // Set previous accelerations to current accelerations
        prev_acc_x = acc_x;
        prev_acc_y = acc_y;
        prev_acc_z = acc_z;
        // Set previous velocities to current velocities
        prev_vel_x = vel_x;
        prev_vel_y = vel_y;
        prev_vel_z = vel_z;

        // Set previous time to current time
        prev_time = current_time;

      

    }

    return 0;
}