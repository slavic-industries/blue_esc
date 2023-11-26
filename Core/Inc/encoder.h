#ifndef ENCODER_H

#include "stm32g4xx_hal.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#ifndef M_2PI
  #define M_2PI 6.2831853071795865
#endif


struct generic_encoder
{
    float meas_pos_cnts;
    float delta_pos_meas_cnts;
    float est_pos_wrap_cnts;
    float vel_est;
    float delta_pos_est_cnts;
    float delta_pos_err;
    float pos_incr;
    float enc_pwm_freq;
    float pwm_period;
    int32_t pos_sector;

    float kp;
    float ki;
    float bw;
    TIM_HandleTypeDef* encoder_htim;
};


void encoder_init(TIM_HandleTypeDef* htim, uint32_t bandwidth);
void encoder_reset(void);

void encoder_update(void);

float encoder_get_pos_cnts(void);
float encoder_get_pos_rad(void);
float encoder_get_pos_deg(void);

float encoder_get_vel_cnts(void);
float encoder_get_vel_rad(void);
float encoder_get_vel_cdeg(void);




#endif