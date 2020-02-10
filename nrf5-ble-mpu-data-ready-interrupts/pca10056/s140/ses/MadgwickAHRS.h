//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include "imu_values.h"

//----------------------------------------------------------------------------------------------------
//Structure declaration

typedef struct {
    double w, x, y, z;
}Quaternion;

typedef struct {
    double roll, pitch, yaw;
}EulerAngles;

typedef struct {
Quaternion q;
imu_values_t a,g;
} imu_data_t;


//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile double beta;				// algorithm gain

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdateIMU(imu_data_t* data);

EulerAngles ToEulerAngles(Quaternion q);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================