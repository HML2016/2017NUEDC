#ifndef IMUSO3_H
#define IMUSO3_H

#define M_PI_F 3.1415926f

typedef struct IMU
{
    float   roll;				//deg
    float   pitch;
    float 	yaw;
    float   rollRad;				//rad
    float   pitchRad;
    float 	yawRad;
} imu_t;

extern volatile imu_t imu;//Å·À­½Ç
/* Function prototypes */
static float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

void IMUSO3Thread(void);

#endif

