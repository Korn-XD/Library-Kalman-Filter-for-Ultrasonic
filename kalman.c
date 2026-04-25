#include "kalman.h"

void Kalman1D_Init(Kalman1D_t *kf, float Q, float R, float P, float initial_value) {
    kf->Q = Q; kf->R = R; kf->P = P; kf->x = initial_value;
}

float Kalman1D_Update(Kalman1D_t *kf, float measurement) {
    kf->P = kf->P + kf->Q;
    kf->K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + kf->K * (measurement - kf->x);
    kf->P = (1 - kf->K) * kf->P;
    return kf->x;
}

void Kalman2D_Init_Kinematic(Kalman2D_t *kf, float dt, float q_pos, float q_vel, float r_pos) {
    arm_mat_init_f32(&kf->x, 2, 1, kf->x_data);
    arm_mat_init_f32(&kf->F, 2, 2, kf->F_data);
    arm_mat_init_f32(&kf->P, 2, 2, kf->P_data);
    arm_mat_init_f32(&kf->Q, 2, 2, kf->Q_data);
    arm_mat_init_f32(&kf->H, 1, 2, kf->H_data);
    arm_mat_init_f32(&kf->R, 1, 1, kf->R_data);
    arm_mat_init_f32(&kf->K, 2, 1, kf->K_data);
    arm_mat_init_f32(&kf->I, 2, 2, kf->I_data);

    // Kinematic F Matrix: [1  dt]
    //                    [0  1 ]
    kf->F_data[0] = 1.0f; kf->F_data[1] = dt;
    kf->F_data[2] = 0.0f; kf->F_data[3] = 1.0f;

    // Process Noise Q Matrix
    kf->Q_data[0] = q_pos; kf->Q_data[1] = 0.0f;
    kf->Q_data[2] = 0.0f;  kf->Q_data[3] = q_vel;

    // Measurement Noise R Matrix
    kf->R_data[0] = r_pos;

    // H Matrix: Measure only position
    kf->H_data[0] = 1.0f; kf->H_data[1] = 0.0f; 

    // Identity and Initial Error Covariance
    kf->I_data[0] = 1.0f; kf->I_data[1] = 0.0f;
    kf->I_data[2] = 0.0f; kf->I_data[3] = 1.0f;
    kf->P_data[0] = 1.0f; kf->P_data[1] = 0.0f;
    kf->P_data[2] = 0.0f; kf->P_data[3] = 1.0f;

    kf->x_data[0] = 0.0f; kf->x_data[1] = 0.0f;
}

void Kalman2D_Update(Kalman2D_t *kf, float measurement) {
    float x_p_data[2], P_p_data[4], Ft_data[4], Ht_data[2], S_data[1], Si_data[1];
    float tmp22_data[4], tmp21_data[2], tmp12_data[2];
    arm_matrix_instance_f32 x_p, P_p, Ft, Ht, S, Si, tmp22, tmp21, tmp12;

    arm_mat_init_f32(&x_p, 2, 1, x_p_data);
    arm_mat_init_f32(&P_p, 2, 2, P_p_data);
    arm_mat_init_f32(&Ft, 2, 2, Ft_data);
    arm_mat_init_f32(&Ht, 2, 1, Ht_data);
    arm_mat_init_f32(&S, 1, 1, S_data);
    arm_mat_init_f32(&Si, 1, 1, Si_data);
    arm_mat_init_f32(&tmp22, 2, 2, tmp22_data);
    arm_mat_init_f32(&tmp21, 2, 1, tmp21_data);
    arm_mat_init_f32(&tmp12, 1, 2, tmp12_data);

    // 1. Predict
    arm_mat_mult_f32(&kf->F, &kf->x, &x_p);
    arm_mat_trans_f32(&kf->F, &Ft);
    arm_mat_mult_f32(&kf->F, &kf->P, &tmp22);
    arm_mat_mult_f32(&tmp22, &Ft, &P_p);
    arm_mat_add_f32(&P_p, &kf->Q, &P_p);

    // 2. Update
    arm_mat_trans_f32(&kf->H, &Ht);
    arm_mat_mult_f32(&kf->H, &P_p, &tmp12);
    arm_mat_mult_f32(&tmp12, &Ht, &S);
    arm_mat_add_f32(&S, &kf->R, &S);
    arm_mat_inverse_f32(&S, &Si);
    arm_mat_mult_f32(&P_p, &Ht, &tmp21);
    arm_mat_mult_f32(&tmp21, &Si, &kf->K);

    float y = measurement - x_p_data[0]; 
    kf->x_data[0] = x_p_data[0] + kf->K_data[0] * y;
    kf->x_data[1] = x_p_data[1] + kf->K_data[1] * y;

    arm_mat_mult_f32(&kf->K, &kf->H, &tmp22);
    arm_mat_sub_f32(&kf->I, &tmp22, &tmp22);
    arm_mat_mult_f32(&tmp22, &P_p, &kf->P);
}