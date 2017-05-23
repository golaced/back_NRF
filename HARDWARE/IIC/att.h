#ifndef _CTRL_H
#define	_CTRL_H

#include "stm32f4xx.h"
#include "include.h"
#include "rc.h"


enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
		PID4,
		PID5,
		PID6,

		PIDITEMS
};



typedef struct
{
	xyz_f_t err;
	xyz_f_t err_old;
	xyz_f_t err_i;
	xyz_f_t eliminate_I;
	xyz_f_t err_d;
	xyz_f_t damp;
	xyz_f_t out;
	pid_t 	PID[PIDITEMS];
	xyz_f_t err_weight;
	float FB;

}ctrl_t;


extern u8 Thr_Low;
extern float Thr_Weight;
extern void GPS_calc_poshold(void);

extern float actual_speed[2];
extern float tar_speed[2];
extern  float  now_position[2], target_position[2];
extern float nav[2];
extern float tar_speed[2];
extern float thr_value;
#define LON 0
#define LAT 1
extern float nav_angle_ukf[2],nav_ukf_g[2];
extern int baro_to_ground;
extern u8 state_v;
#endif

