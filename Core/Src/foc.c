#include "foc.h"


static struct foc_data controller;

extern TIM_TypeDef* htim6;

void init_foc(struct foc_data* data)
{
    controller.pos_m_ref = 0.0;
    controller.pos_m_est = 0.0;
    controller.pos_m_err = 0.0;
    controller.pos_m_kp = 0.0;
    
    controller.vel_ref = 0.0;
    controller.vel_est = 0.0;
    controller.vel_err = 0.0;
    controller.vel_ff = 0.0;
    controller.vel_kp = 0.0;
    controller.vel_ki = 0.0;
    controller.vel_sum = 0.0;
    controller.iq_ref = 0.0;
    controller.iq_est = 0.0;
    controller.iq_err = 0.0;
    controller.iq_ff = 0.0;
    controller.iq_kp = 0.0;
    controller.iq_ki = 0.0;
    controller.iq_sum = 0.0;
    controller.iq_integrator = 0.0f;

    controller.id_ref = 0.0;
    controller.id_est = 0.0;
    controller.id_err = 0.0;
    controller.id_ff = 0.0;
    controller.id_kp = 0.0;
    controller.id_ki = 0.0;
    controller.id_sum = 0.0;
    controller.id_integrator = 0.0f;

    controller.v_q = 0.0;
    controller.v_d = 0.0;
    controller.v_alpha = 0.0;
    controller.v_beta = 0.0;
    controller.v_a = 0.0;
    controller.v_b = 0.0;
    controller.v_c = 0.0;
    controller.i_a = 0.0;
    controller.i_b = 0.0;
    controller.i_c = 0.0;
    controller.i_alpha = 0.0;
    controller.i_beta = 0.0;

    controller.pos_e_est = 0.0;
    
    controller.sqrt_3 = sqrt(3);

    controller.t_delta = 0;
    controller.t_prev = 0;
}

void current_foc_update()
{   
    const uint32_t t_now = htim6->CNT;
    controller.t_delta = t_now - controller.t_prev;
    controller.t_prev = t_now;

    controller.iq_err = controller.iq_ref + controller.iq_ff - controller.iq_est;
    controller.id_err = controller.id_ref - controller.id_est;

    controller.iq_sum += controller.iq_err;
    controller.id_sum += controller.id_err;

    controller.iq_integrator += controller.iq_err * controller.t_delta * controller.iq_ki;
    controller.id_integrator += controller.id_err * controller.t_delta * controller.id_ki;

    controller.v_q = (controller.iq_err * controller.iq_kp) + controller.iq_integrator;
    controller.v_d = (controller.id_err * controller.id_kp) + controller.id_integrator;
}

void clarke_transform()
{
    controller.i_alpha = controller.i_a;
    controller.i_beta = (controller.i_a + 2*controller.i_b) / controller.sqrt_3;
}

void park_transform()
{
    float cos_pos_e = cosf(controller.pos_e_est);
    float sin_pos_e = sinf(controller.pos_e_est);

    controller.id_est = controller.i_alpha*cos_pos_e + controller.i_beta*sin_pos_e;
    controller.iq_est = -controller.i_alpha*sin_pos_e + controller.i_beta*cos_pos_e;
}

void inverse_clarke_transrorm()
{
    controller.v_a = controller.v_beta;
    controller.v_b = (-controller.v_beta + controller.sqrt_3*controller.v_alpha) / 2.0f;
    controller.v_c = (-controller.v_beta - controller.sqrt_3*controller.v_alpha) / 2.0f;
}

void inverse_park_transrorm()
{
    float cos_pos_e = cosf(controller.pos_e_est);
    float sin_pos_e = sinf(controller.pos_e_est);

    controller.v_alpha = controller.v_d * cos_pos_e - controller.v_q*sin_pos_e;
    controller.v_beta =  controller.v_d * sin_pos_e + controller.v_q*cos_pos_e;
}