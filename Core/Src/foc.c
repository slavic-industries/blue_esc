#include "foc.h"
#include "main.h"

static struct foc_data controller;

volatile uint16_t ADC1_DMA[5] = {0, 0, 0, 0, 0}; // Dummy conversion (ST workaround for -x),
volatile uint16_t ADC2_DMA[3] = {0, 0, 0};       // Dummy conversion (ST workaround for -x)
static float motor_current_input_adc_offset[3] = {2464.0f,2482.0f,2485.0f};
static int32_t current_samples = 0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

// extern GPIO_TypeDef * STATUS_GPIO_Port;
// extern uint16_t STATUS_Pin;

void init_foc()
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

    controller.motor_bus_voltage = 0.0f;
    controller.alpha_voltage = 0.01f;
    controller.motor_bus_voltage_adc = 0.0f;

    controller.motor_current_input_adc[0] = 0;
    controller.motor_current_input_adc[1] = 0;
    controller.motor_current_input_adc[2] = 0;

    controller.sqrt_3 = sqrt(3.0f);

    controller.t_delta = 0;
    controller.t_prev = 0;

    // Motor PWM init and BRAKE
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    // OPAMP and ADC init
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
    HAL_ADC_Start_DMA(&hdma_adc1, (uint32_t *)ADC1_DMA, 5);
    HAL_ADC_Start_DMA(&hdma_adc2, (uint32_t *)ADC2_DMA, 3);

    // HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
}

void current_foc_update()
{
    const uint32_t t_now = (htim6).Instance->CNT;
    controller.t_delta = t_now - controller.t_prev;
    controller.t_prev = t_now;

    // Measure phase current using the ADC

    clarke_transform();
    park_transform();

    controller.iq_err = controller.iq_ref + controller.iq_ff - controller.iq_est;
    controller.id_err = controller.id_ref - controller.id_est;

    controller.iq_sum += controller.iq_err;
    controller.id_sum += controller.id_err;

    controller.iq_integrator += controller.iq_err * controller.t_delta * controller.iq_ki;
    controller.id_integrator += controller.id_err * controller.t_delta * controller.id_ki;

    controller.v_q = (controller.iq_err * controller.iq_kp) + controller.iq_integrator;
    controller.v_d = (controller.id_err * controller.id_kp) + controller.id_integrator;

    inverse_park_transrorm();
    inverse_clarke_transrorm();

    // PWSVM
    pwm_generation();
}

void clarke_transform()
{
    controller.i_alpha = controller.i_a;
    controller.i_beta = (controller.i_a + 2 * controller.i_b) / controller.sqrt_3;
}

void park_transform()
{
    float cos_pos_e = cosf(controller.pos_e_est);
    float sin_pos_e = sinf(controller.pos_e_est);

    controller.id_est = controller.i_alpha * cos_pos_e + controller.i_beta * sin_pos_e;
    controller.iq_est = -controller.i_alpha * sin_pos_e + controller.i_beta * cos_pos_e;
}

void inverse_clarke_transrorm()
{
    controller.v_a = controller.v_beta;
    controller.v_b = (-controller.v_beta + controller.sqrt_3 * controller.v_alpha) / 2.0f;
    controller.v_c = (-controller.v_beta - controller.sqrt_3 * controller.v_alpha) / 2.0f;
}

void inverse_park_transrorm()
{
    float cos_pos_e = cosf(controller.pos_e_est);
    float sin_pos_e = sinf(controller.pos_e_est);

    controller.v_alpha = controller.v_d * cos_pos_e - controller.v_q * sin_pos_e;
    controller.v_beta = controller.v_d * sin_pos_e + controller.v_q * cos_pos_e;
}

void pwm_generation()
{
    // Apply CSVPWM to v_a, v_b and v_c
    const float v_neutral = 0.5f * (fmaxf(fmaxf(controller.v_a, controller.v_b), controller.v_a) + fminf(fminf(controller.v_a, controller.v_b), controller.v_c));
}

void motor_bus_voltage_calculation()
{
    // process input voltage (STM32G431-ESC1 specific)
    static float const R68 = 169.0f; // kohm
    static float const R76 = 18.0f;  // kohm
    // static float const alpha_voltage = 0.01f;
    controller.motor_bus_voltage = (controller.motor_bus_voltage_adc / 4096.0f * 3.3f * (R68 + R76) / R76) * controller.alpha_voltage + (1.0f - controller.alpha_voltage) * controller.motor_bus_voltage;
}

void adc_interrupt(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
        {
            // Filter (EWMA) position and voltage ADC samples
            controller.motor_current_input_adc[0] = ADC1_DMA[1];
            // potentiometer_input_adc = ADC1_DMA[2];
            controller.motor_bus_voltage_adc = ((float)ADC1_DMA[3]);
            // temperature_input_adc = ADC1_DMA[4];
        }
        else
        {
            motor_current_input_adc_offset[0] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC1_DMA[1]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[0];
        }
        // restart ADC
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_DMA, 5);
    }
    if (hadc == &hadc2)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
        {
            // Filter (EWMA) position and voltage ADC samples
            controller.motor_current_input_adc[1] = ADC2_DMA[1];
            controller.motor_current_input_adc[2] = ADC2_DMA[2];
            ++current_samples;
        }
        else
        {
            motor_current_input_adc_offset[1] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC2_DMA[1]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[1];
            motor_current_input_adc_offset[2] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC2_DMA[2]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[2];
        }
        // restart ADC
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_DMA, 3);
    }
}