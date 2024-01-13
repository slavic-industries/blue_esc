#ifndef FOC_H
#define FOC_H

#include "stm32g4xx_hal.h"
#include "math.h"

#define MIN_PWM_DUTY_CYCLE 0.02f
#define MAX_PWM_DUTY_CYCLE 0.98f

#define ALPHA_CURRENT_SENSE_OFFSET 0.001f


struct foc_data
{
    float pos_m_ref;
    float pos_m_est;
    float pos_m_err;
    float pos_m_kp;

    float vel_ref;
    float vel_est;
    float vel_err;
    float vel_ff;
    float vel_kp;
    float vel_ki;
    float vel_sum;

    float iq_ref;
    float iq_est;
    float iq_err;
    float iq_ff;
    float iq_kp;
    float iq_ki;
    float iq_sum;
    float iq_integrator;

    float id_ref;
    float id_est;
    float id_err;
    float id_ff;
    float id_kp;
    float id_ki;
    float id_sum;
    float id_integrator;

    float v_q;
    float v_d;

    float v_alpha;
    float v_beta;

    float v_a;
    float v_b;
    float v_c;

    float i_a;
    float i_b;
    float i_c;

    float i_alpha;
    float i_beta;

    float pos_e_est;

    float motor_bus_voltage;
    float alpha_voltage;
    float motor_bus_voltage_adc;

    uint16_t motor_current_input_adc[3];

    // Constants
    float sqrt_3;

    uint32_t t_delta;
    uint32_t t_prev;
};

void clarke_transform();
void park_transform();

void inverse_clarke_transrorm();
void inverse_park_transrorm();

void init_foc();
void current_foc_update();

void current_measurement();
void pwm_generation();

void motor_bus_voltage_calculation();

void adc_interrupt(ADC_HandleTypeDef *hadc);
#endif