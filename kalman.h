#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "arm_math.h"

typedef struct {
    float Q; float R; float x; float P; float K;
} Kalman1D_t;

typedef struct {
    arm_matrix_instance_f32 x, F, P, Q, H, R, K, I;
    float x_data[2], F_data[4], P_data[4], Q_data[4], H_data[2], R_data[1], K_data[2], I_data[4];
} Kalman2D_t;

void Kalman1D_Init(Kalman1D_t *kf, float Q, float R, float P, float initial_value);
float Kalman1D_Update(Kalman1D_t *kf, float measurement);

void Kalman2D_Init_Kinematic(Kalman2D_t *kf, float dt, float q_pos, float q_vel, float r_pos);
void Kalman2D_Update(Kalman2D_t *kf, float measurement);

#endif /* INC_KALMAN_H_ */