#include "encoder.h"

#define TIM_4_FREQUENCY 160000000 / 3.0
#define COUNTS_PER_FULL_REVOLUTION 4096.0
#define COUNTS_PER_HALF_REVOLUTION 2048.0


static float const cnts2rad = 2*M_PI/4096.0f;
static float const cnts2deg = 360.0/4096.0f;


struct generic_encoder encoder;


void encoder_init(TIM_HandleTypeDef* htim, uint32_t bandwdith)
{
    encoder.encoder_htim = htim;
    encoder.meas_pos_cnts = 0.0;
    encoder.delta_pos_meas_cnts = 0.0;
    encoder.est_pos_wrap_cnts = 0.0;
    encoder.vel_est = 10.010;
    encoder.delta_pos_est_cnts = 0.0;
    encoder.delta_pos_err = 0.0;
    encoder.pos_incr = 0.0;
    encoder.enc_pwm_freq = 0.0;
    encoder.pwm_period = 0.0;
    encoder.pos_sector = 0;

    encoder.bw = bandwdith;
    encoder.kp = 2.0f * encoder.bw;
    encoder.ki = 0.25f * encoder.kp * encoder.kp;
    HAL_TIM_IC_Start_IT(encoder.encoder_htim, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(encoder.encoder_htim, TIM_CHANNEL_2);
}

void encoder_update()
{  
    uint32_t cl = HAL_TIM_ReadCapturedValue(encoder.encoder_htim, TIM_CHANNEL_1);
	uint32_t ch = HAL_TIM_ReadCapturedValue(encoder.encoder_htim, TIM_CHANNEL_2);
    encoder.enc_pwm_freq = TIM_4_FREQUENCY / cl;
    encoder.pwm_period = 1 / encoder.enc_pwm_freq;

    encoder.meas_pos_cnts = COUNTS_PER_FULL_REVOLUTION * ch / cl;
    encoder.delta_pos_meas_cnts =encoder.meas_pos_cnts - encoder.est_pos_wrap_cnts;
    if(encoder.delta_pos_meas_cnts < -COUNTS_PER_HALF_REVOLUTION)
    {
      encoder.delta_pos_meas_cnts += COUNTS_PER_FULL_REVOLUTION;
    }
    else if(encoder.delta_pos_meas_cnts >= COUNTS_PER_HALF_REVOLUTION)
    {
      encoder.delta_pos_meas_cnts -= COUNTS_PER_FULL_REVOLUTION;
    }
    encoder.delta_pos_est_cnts  = encoder.pwm_period * encoder.vel_est;
    encoder.delta_pos_err  = encoder.delta_pos_meas_cnts - encoder.delta_pos_est_cnts;
    encoder.pos_incr = encoder.delta_pos_est_cnts + (encoder.kp  * encoder.pwm_period * encoder.delta_pos_err);
    encoder.est_pos_wrap_cnts += encoder.pos_incr;
    if(encoder.est_pos_wrap_cnts < 0)
    {
      encoder.est_pos_wrap_cnts += COUNTS_PER_FULL_REVOLUTION;
      encoder.pos_sector -= 1;
    }
    else if(encoder.est_pos_wrap_cnts >= COUNTS_PER_FULL_REVOLUTION)
    {
      encoder.est_pos_wrap_cnts -= COUNTS_PER_FULL_REVOLUTION;
      encoder.pos_sector += 1;
    }
    encoder.vel_est += encoder.pwm_period * encoder.ki * encoder.delta_pos_err;
}

float encoder_get_pos_cnts()
{
    return encoder.est_pos_wrap_cnts;
}


float encoder_get_pos_rad()
{
    return encoder.est_pos_wrap_cnts * cnts2rad;
}

float encoder_get_pos_deg()
{
    return encoder.est_pos_wrap_cnts * cnts2deg;
}

float encoder_get_vel_cnts()
{
    return encoder.vel_est;
}

float encoder_get_vel_rad()
{
    return encoder.vel_est * cnts2rad;
}

float encoder_get_vel_deg()
{
    return encoder.vel_est * cnts2deg;
}