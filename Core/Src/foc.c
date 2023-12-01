#include "foc.h"
#include "math.h"

const struct foc_data controller;

void clarke_transform(struct foc_data* data)
{
    data->i_alpha = data->i_a;
    data->i_beta = (data->i_a + 2*data->i_b) / data->sqrt_3;
}

void park_transform(struct foc_data* data)
{
    float cos_pos_e = cosf(data->pos_e_est);
    float sin_pos_e = sinf(data->pos_e_est);

    data->id_est = data->i_alpha*cos_pos_e + data->i_beta*sin_pos_e;
    data->iq_est = -data->i_alpha*sin_pos_e + data->i_beta*cos_pos_e;
}