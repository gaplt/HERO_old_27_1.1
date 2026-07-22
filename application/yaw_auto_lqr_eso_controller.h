/**
 ******************************************************************************
 * @file    yaw_auto_lqr_eso_controller.h
 * @school  湖北工业大学
 * @author  董阳星
 * @email   3259404430@qq.com
 * @date    2026/7/19
 * @brief   Standalone yaw auto-aim LQR-ESO controller matching the math summary.
 *
 * This pair is a documentation companion for yaw-auto-lqreso-math-summary.docx.
 * It keeps only the continuous-time reference tracking, feedforward, LQR,
 * third-order ESO, disturbance compensation, torque limit, and slew limit chain.
 ******************************************************************************
 */

#ifndef YAW_AUTO_LQR_ESO_CONTROLLER_H
#define YAW_AUTO_LQR_ESO_CONTROLLER_H

#include <stdint.h>

#define J 0.0514f
#define B 0.0757f
#define K_Theta  18.1659f
#define K_Omega  3.3700f
#define K_i        0.0f            
#define Theta_integral_limit_rad_s 0.8f 
#define Tau_coulomb_nm     0.07f                   //库仑摩擦
#define Coulomb_smooth_rad_s    0.15f             //0.15      ws  大概率是定值
#define Eso_bandwidth_rad_s        10.0f         //调试得到   5.0   w0
#define Eso_comp_gain      1.0f               //调试得到   5.0  kESO
#define Eso_comp_limit_nm      10000.0f
#define Eso_omega_gate_rad_s   8000.0f
#define Eso_alpha_gate_rad_s2  8000.0f
#define Tau_bias_ki            800.0f     
#define Tau_bias_limit_nm      10000.0f
#define Tau_meas_lpf_alpha     1.0f
#define Theta_deadband_rad     0.0f
#define Torque_soft_limit_nm   0.0f
#define Torque_min_nm         -16384.0f
#define Torque_max_nm          16384.0f
#define Torque_slew_rate_nm_s  0.0f
//#define Eso_enable             1U




#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float j_kg_m2;
    float b_nms_rad;
    float k_theta;
    float k_omega;
    float k_i;
    float theta_integral_limit_rad_s;
    float tau_coulomb_nm;
    float coulomb_smooth_rad_s;
    float eso_bandwidth_rad_s;
    float eso_comp_gain;
    float eso_comp_limit_nm;
    float eso_omega_gate_rad_s;
    float eso_alpha_gate_rad_s2;
    float tau_bias_ki;
    float tau_bias_limit_nm;
    float tau_meas_lpf_alpha;
    float theta_deadband_rad;
    float torque_soft_limit_nm;
    float torque_min_nm;
    float torque_max_nm;
    float torque_slew_rate_nm_s;
    uint8_t eso_enable;
    uint8_t eso_comp_enable;          //自抗扰观测器
    uint8_t torque_slew_enable;       //力矩斜率开关
} YawAutoLqrEsoConfig_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
} YawAutoLqrEsoState_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float alpha_rad_s2;
} YawAutoLqrEsoReference_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    uint8_t feedback_ok;
} YawAutoLqrEsoFeedback_t;

typedef struct
{
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
} YawAutoLqrEsoObserver_t;

typedef struct
{
    YawAutoLqrEsoObserver_t eso;
    float theta_integral_rad_s;
    float tau_bias_nm;
    float tau_meas_filt_nm;
    float tau_cmd_last_nm;
    uint8_t feedback_ready;
} YawAutoLqrEso_t;

typedef struct
{
    float a[2][2];
    float b[2][2];   /* input vector is [tau, d] */
    float c[2];
} YawAutoLqrEsoStateSpace_t;

typedef struct
{
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_ff_alpha_nm;
    float tau_ff_viscous_nm;
    float tau_ff_coulomb_nm;
    float tau_ff_nm;
    float tau_i_nm;
    float tau_lqr_nm;
    float tau_eso_raw_nm;
    float tau_eso_active_nm;
    float tau_bias_nm;
    float tau_pre_limit_nm;
    float tau_cmd_before_slew_nm;
    float tau_cmd_nm;
    uint8_t eso_comp_active;
    uint8_t soft_limit_active;
    uint8_t hard_limit_active;
    uint8_t slew_limit_active;
} YawAutoLqrEsoOutput_t;

extern void YawAutoLqrEso_Init(YawAutoLqrEso_t *ctrl);
extern void YawAutoLqrEsoConfig_Init(YawAutoLqrEsoConfig_t *cfg);
extern void YawAutoLqrEso_Reset(YawAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s);
extern uint8_t YawAutoLqrEso_GetStateSpace( YawAutoLqrEsoConfig_t *cfg,
                                    YawAutoLqrEsoStateSpace_t *model);
extern void YawAutoLqrEso_Calc(YawAutoLqrEso_t *ctrl,
                         YawAutoLqrEsoConfig_t *cfg,
                         YawAutoLqrEsoFeedback_t *feedback,
                         YawAutoLqrEsoReference_t *ref,
                        float dt_s,
                        YawAutoLqrEsoOutput_t *output);


#ifdef __cplusplus
}
#endif

#endif
