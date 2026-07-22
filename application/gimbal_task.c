#include "gimbal_task.h"
//#include "MOTOR_ANGLE_PID.h"
#include "bsp_usb.h"
#include "kalman.h"
#include "robomaster_vcan.h"
#include "user_lib.h"
#include <math.h>
#include "bsp_usart.h"
#include "usart.h"
#include "usb_task.h"
#include "cmd_ctr.h"
#include "stm32_flash.h"
#include "protocol.h"
#include "yaw_auto_lqr_eso_controller.h"
#include "DWT.h"
int yaw_offset_ecd  =  4250;
fp32 yaw_rc_multiple ;
fp32 pitch_rc_multiple;
fp32 YAW_MOUSE_SEN ;
fp32 PITCH_MOUSE_SEN;

uint32_t DWT_t = 0;
float Dt = 0;
int gimbal_yaw_visual = 0;
float gimbal_yaw_visual_tau_cmd;      //视觉yaw目标力矩

#define YAW_STEP_SAMPLE_FREQ_HZ       500.0f
#define YAW_STEP_DEFAULT_AMPLITUDE_A  0.20f
#define YAW_STEP_MAX_CURRENT_A        3.00f
#define YAW_STEP_MAX_SPEED_RAD_S      8.00f

float gimbal_test_switch = 2;      //选择采样模式，0为阶跃信号，1为扫频信号,2为关闭


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

gimbal gimbal_ctrl;

void gimbal_init(gimbal *gimbal_init);
void gimbal_loop(gimbal *loop);
static void gimbal_feedback_update(gimbal *feedback_update);
static void gimbal_movement(gimbal *movement);
		uint8_t visual_use_date[2][20];
		uint8_t send_date[100];
		
void gimbal_task(void const *pvParameters)
{
	STMFLASH_Read(DATE_START_ADD, (uint32_t *)&save_date, sizeof(save_date));
	gimbal_init(&gimbal_ctrl);

	for(;;)
	{
     
		 gimbal_feedback_update(&gimbal_ctrl);
     gimbal_loop(&gimbal_ctrl);
//		usb_printf("%f,%f,%f,%f\n", gimbal_ctrl.gimbal_pitch_motor.absolute_angle, gimbal_ctrl.gimbal_pitch_motor.absolute_angle_set ,gimbal_ctrl.gimbal_pitch_motor.motor_gyro ,gimbal_ctrl.gimbal_pitch_motor.motor_gyro_set );

		usb_printf("%f,%f,%f,%f\n", gimbal_ctrl.gimbal_yaw_motor.absolute_angle, gimbal_ctrl.gimbal_yaw_motor.absolute_angle_set ,gimbal_ctrl.gimbal_yaw_motor.motor_gyro ,gimbal_ctrl.gimbal_yaw_motor.motor_gyro_set );

	   vTaskDelay(5);

	}

}


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}


// 阶跃信号状态结构体
typedef struct
{
    uint8_t running;        // 是否正在运行
    uint8_t finished;       // 是否完成
    uint8_t fault;          // 故障标志
    uint8_t phase;          // 0=正阶跃, 1=负阶跃, 2=停止
    
    uint32_t step_index;    // 当前阶跃次数
    uint32_t total_steps;   // 总阶跃次数
    
    float sample_frequency_hz;
    float current_amplitude_a;
    float current_command_a;
    
    uint32_t samples_per_step;      // 每个阶跃的采样点数
    uint32_t sample_index;          // 当前采样索引
} YawStepState_t;

static YawStepState_t yaw_step;

// 阶跃信号幅值表（可自定义每个阶跃的幅值）
static const float step_amplitude_table[] = {
    0.20f,   // 第1步：正0.2A
   -0.20f,   // 第2步：负0.2A
    0.40f,   // 第3步：正0.4A
   -0.40f,   // 第4步：负0.4A
    0.60f,   // 第5步：正0.6A
   -0.60f,   // 第6步：负0.6A
    0.80f,   // 第7步：正0.8A
   -0.80f,   // 第8步：负0.8A
    1.00f,   // 第9步：正1.0A
   -1.00f,   // 第10步：负1.0A
	 1.20f,   // 第11步：正1.2A
   -1.20f,   // 第12步：负1.2A
	 1.40f,   // 第13步：正1.4A
   -1.40f,   // 第14步：负1.4A
	1.60f,   // 第15步：正1.6A
   -1.60f,   // 第16步：负1.6A
	1.80f,   // 第17步：正1.8A
   -1.80f,   // 第18步：负1.8A
	2.00f,   // 第19步：正2.0A
   -2.00f,   // 第20步：负2.0A
	2.20f,   // 第21步：正2.2A
   -2.20f,   // 第22步：负2.2A
};

#define YAW_STEP_COUNT \
(sizeof(step_amplitude_table) / sizeof(step_amplitude_table[0]))

// 每个阶跃持续的时间（秒）
#define YAW_STEP_DURATION_SEC      2.0f
#define YAW_STEP_IDLE_SEC          1.0f   // 阶跃之间的等待时间

static void YawStepPrepareNextStep(void)
{
    float amplitude;
    
    if (yaw_step.step_index >= YAW_STEP_COUNT)
    {
        // 所有阶跃完成
        yaw_step.running = 0U;
        yaw_step.finished = 1U;
        yaw_step.current_command_a = 0.0f;
        yaw_step.phase = 2U;  // 停止
        return;
    }
    
    // 获取当前阶跃的幅值
    amplitude = step_amplitude_table[yaw_step.step_index];
    
    // 限制最大电流
    if (amplitude > YAW_STEP_MAX_CURRENT_A)
        amplitude = YAW_STEP_MAX_CURRENT_A;
    else if (amplitude < -YAW_STEP_MAX_CURRENT_A)
        amplitude = -YAW_STEP_MAX_CURRENT_A;
    
    yaw_step.current_amplitude_a = amplitude;
    yaw_step.sample_index = 0U;
    
    // 计算每个阶跃的采样点数
    yaw_step.samples_per_step = (uint32_t)(YAW_STEP_DURATION_SEC * yaw_step.sample_frequency_hz);
    
    // 判断当前阶跃的方向
    if (amplitude > 0.0f)
        yaw_step.phase = 0U;   // 正阶跃
    else if (amplitude < 0.0f)
        yaw_step.phase = 1U;   // 负阶跃
    else
        yaw_step.phase = 2U;   // 停止
    
    // 立即设置电流指令
    yaw_step.current_command_a = amplitude;
}

static void YawStepStart(float amplitude_a, uint32_t total_steps)
{
    memset(&yaw_step, 0, sizeof(yaw_step));
    
    yaw_step.sample_frequency_hz = YAW_STEP_SAMPLE_FREQ_HZ;
    yaw_step.total_steps = total_steps;
    yaw_step.step_index = 0U;
    yaw_step.running = 1U;
    yaw_step.finished = 0U;
    yaw_step.fault = 0U;
    
    YawStepPrepareNextStep();
}

static void YawStepStop(uint8_t fault)
{
    yaw_step.running = 0U;
    yaw_step.current_command_a = 0.0f;
    
    if (fault != 0U)
    {
        yaw_step.fault = 1U;
        yaw_step.finished = 0U;
    }
    else
    {
        yaw_step.fault = 0U;
        yaw_step.finished = 1U;
    }
}

static float YawStepUpdate(void)
{
    if (yaw_step.running == 0U)
    {
        yaw_step.current_command_a = 0.0f;
        return 0.0f;
    }
    
    // 增加采样计数
    yaw_step.sample_index++;
    
    // 检查当前阶跃是否完成
    if (yaw_step.sample_index >= yaw_step.samples_per_step)
    {
        // 当前阶跃完成，进入下一个
        yaw_step.step_index++;
        YawStepPrepareNextStep();
    }
    
    return yaw_step.current_command_a;
}

// 获取当前阶跃的幅值（用于数据记录）
static float YawStepGetCurrentAmplitude(void)
{
    return yaw_step.current_amplitude_a;
}

// 获取阶跃状态（0=正阶跃, 1=负阶跃, 2=停止）
static uint8_t YawStepGetPhase(void)
{
    return yaw_step.phase;
}

#define YAW_SWEEP_SAMPLE_FREQ_HZ       500.0f
#define YAW_SWEEP_CYCLES_PER_FREQ      20U

#define YAW_SWEEP_DEFAULT_AMPLITUDE_A  0.20f
#define YAW_SWEEP_MAX_CURRENT_A        1.00f
#define YAW_SWEEP_MAX_SPEED_RAD_S      8.00f

static const float yaw_sweep_frequency_table[] = {
    1.0f,  2.0f, 3.0f,
     4.0f,  5.0f,

    6.0f, 7.0f, 8.0f, 9.0f, 10.0f,
      20.0f,

     30.0f,
     40.0f,  50.0f
};

#define YAW_SWEEP_FREQ_COUNT \
(sizeof(yaw_sweep_frequency_table) / \
sizeof(yaw_sweep_frequency_table[0]))

#define YAW_SWEEP_MIN_ANGLE_ANG   (-1.5f)
#define YAW_SWEEP_MAX_ANGLE_ANG   ( 1.5f)

//采样变量
float current_cmd;
float current_set, current_m,current_t, w_m, angle_m, imu_angle_speed, imu_angle, is_sweep,is_step, real_hz;

typedef struct
{
    uint8_t running;
    uint8_t finished;
    uint8_t fault;

    uint16_t frequency_index;

    uint32_t samples_per_cycle;
    uint32_t total_samples_in_segment;
    uint32_t segment_sample_index;

    uint32_t cycles_per_frequency;

    float sample_frequency_hz;
    float current_amplitude_a;

    float actual_frequency_hz;
    float current_command_a;
} YawSweepState_t;

static YawSweepState_t yaw_sweep;

static void YawSweepPrepareCurrentFrequency(void)
{
    float target_frequency;
    float samples_float;
    uint32_t samples_per_cycle;

    if (yaw_sweep.frequency_index >= YAW_SWEEP_FREQ_COUNT)
    {
        yaw_sweep.running = 0U;
        yaw_sweep.finished = 1U;
        yaw_sweep.current_command_a = 0.0f;
        return;
    }

    target_frequency =
        yaw_sweep_frequency_table[yaw_sweep.frequency_index];

    samples_float =
        yaw_sweep.sample_frequency_hz / target_frequency;

    samples_per_cycle =
        (uint32_t)(samples_float + 0.5f);

    /*
     * 1 kHz 控制频率下至少保留 10 个采样点/周期。
     * 这样最高实际频率不会超过 100 Hz。
     */
    if (samples_per_cycle < 10U)
    {
        samples_per_cycle = 10U;
    }

    yaw_sweep.samples_per_cycle = samples_per_cycle;

    yaw_sweep.actual_frequency_hz =
        yaw_sweep.sample_frequency_hz /
        (float)yaw_sweep.samples_per_cycle;

    yaw_sweep.total_samples_in_segment =
        yaw_sweep.samples_per_cycle *
        yaw_sweep.cycles_per_frequency;

    yaw_sweep.segment_sample_index = 0U;
}

static void YawSweepStart(float amplitude_a,
                          uint32_t cycles_per_frequency)
{
    memset(&yaw_sweep, 0, sizeof(yaw_sweep));

    if (amplitude_a > YAW_SWEEP_MAX_CURRENT_A)
    {
        amplitude_a = YAW_SWEEP_MAX_CURRENT_A;
    }
    else if (amplitude_a < -YAW_SWEEP_MAX_CURRENT_A)
    {
        amplitude_a = -YAW_SWEEP_MAX_CURRENT_A;
    }

    yaw_sweep.sample_frequency_hz =
        YAW_SWEEP_SAMPLE_FREQ_HZ;

    yaw_sweep.current_amplitude_a =
        fabsf(amplitude_a);

    yaw_sweep.cycles_per_frequency =
        cycles_per_frequency;

    yaw_sweep.frequency_index = 0U;
    yaw_sweep.running = 1U;
    yaw_sweep.finished = 0U;
    yaw_sweep.fault = 0U;

    YawSweepPrepareCurrentFrequency();
}

static float YawSweepGetAmplitude(float frequency_hz)
{
    if (frequency_hz < 2.0f)
    {
        return 0.4f;
    }
    else if (frequency_hz <= 10.0f)
    {
        return 0.85f;
    }
    else
    {
        return 1.0f;
    }
}

static void YawSweepStop(uint8_t fault)
{
    yaw_sweep.running = 0U;
    yaw_sweep.current_command_a = 0.0f;

    if (fault != 0U)
    {
        yaw_sweep.fault = 1U;
        yaw_sweep.finished = 0U;
    }
    else
    {
        yaw_sweep.fault = 0U;
        yaw_sweep.finished = 1U;
    }
}

static float YawSweepUpdate(void)
{
    uint32_t sample_in_cycle;
    float phase;

    if (yaw_sweep.running == 0U)
    {
        yaw_sweep.current_command_a = 0.0f;
        return 0.0f;
    }

    sample_in_cycle =
        yaw_sweep.segment_sample_index %
        yaw_sweep.samples_per_cycle;

    phase =
        2.0f * PI *
        (float)sample_in_cycle /
        (float)yaw_sweep.samples_per_cycle;

    yaw_sweep.current_command_a =
        yaw_sweep.current_amplitude_a *
        sinf(phase);

    yaw_sweep.segment_sample_index++;

    if (yaw_sweep.segment_sample_index >=
        yaw_sweep.total_samples_in_segment)
    {
        yaw_sweep.frequency_index++;
        YawSweepPrepareCurrentFrequency();
    }

    return yaw_sweep.current_command_a;
}



/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
	* @author         RM
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal"变量指针.
	* @author         刘根 
  * @retval         none
  */
static void gimbal_feedback_update(gimbal *feedback_update)
{
	    if (feedback_update == NULL)
    {
        return;
    }
			
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                      feedback_update->gimbal_pitch_motor.offset_ecd);
		feedback_update->gimbal_pitch_motor.motor_gyro =  *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);    //更新陀螺仪角度
	
		feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

}

static void gimbal_movement(gimbal *movement)   //控制云台角度
{
	  static int16_t yaw_channel = 0, pitch_channel = 0;	
	  rc_deadband_limit(movement->gimbal_rc->rc.ch[gimbal_yaw_CHANNEL], yaw_channel, 10);
		rc_deadband_limit(movement->gimbal_rc->rc.ch[gimbal_pitch_CHANNEL], pitch_channel, 10);
	  first_order_filter_cali(&movement->gimbal_cmd_slow_set_x,movement->gimbal_rc->mouse.x);
	  first_order_filter_cali(&movement->gimbal_cmd_slow_set_y,movement->gimbal_rc->mouse.y);
	
  	movement->gimbal_pitch_motor.absolute_angle_set -= pitch_channel*pitch_rc_multiple +movement->gimbal_rc->mouse.y * PITCH_MOUSE_SEN;	  
	  
  	movement->gimbal_yaw_motor.absolute_angle_set   -= yaw_channel*yaw_rc_multiple +movement->gimbal_rc->mouse.x * YAW_MOUSE_SEN;  
	  if(movement->gimbal_yaw_motor.absolute_angle_set>pi)  //遥控器控制加减回零点
		{
			
			movement->gimbal_yaw_motor.absolute_angle_set=-pi;
		
		}
		else if(movement->gimbal_yaw_motor.absolute_angle_set<-pi)
		{
		  movement->gimbal_yaw_motor.absolute_angle_set=pi;
		}
		
		if(movement->gimbal_pitch_motor.absolute_angle_set>=MAX_ANGLE)  //限幅
		{
			movement->gimbal_pitch_motor.absolute_angle_set = MAX_ANGLE;
		
		}
		else if(movement->gimbal_pitch_motor.absolute_angle_set<=MIN_ANGLE)
		{
			movement->gimbal_pitch_motor.absolute_angle_set = MIN_ANGLE;
		
		}
		
		
		
}

/**
  * @brief         返回云台yaw轴电机数据指针           
  * @param[in]     void
	* @author        刘根 
  * @retval        gimbal_ctrl.gimbal_yaw_motor
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_ctrl.gimbal_yaw_motor;
}

/**
  * @brief         云台初始化函数          
  * @param[in]     gimbal整体云台参数结构体 
	* @author        刘根 
  * @retval        none 
  */
void gimbal_init(gimbal *gimbal_init)
{
	if (gimbal_init == NULL)
	{
			return;
	}
  flash_date_init();
	const static fp32 gimbal_x_order_filter[1] = {GIMBAL_ACCEL_X_NUM};    //速度设置的一阶滤波参数
	const static fp32 gimbal_y_order_filter[1] = {GIMBAL_ACCEL_Y_NUM};
	
	first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_x,CHASSIS_CONTROL_TIME,gimbal_x_order_filter);
	first_order_filter_init(&gimbal_init->gimbal_cmd_slow_set_y,CHASSIS_CONTROL_TIME,gimbal_y_order_filter);
	
	gimbal_init->first_angle_point = 0;
	gimbal_init->yaw_first_angle_flag = 0;
	gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_ZERO;
	gimbal_init->gimbal_pitch_motor.max_relative_angle=MAX_ANGLE;//机械角度限幅
	gimbal_init->gimbal_pitch_motor.min_relative_angle=MIN_ANGLE;
	
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();  //yaw           
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();  //pitch
  gimbal_init->gimbal_rc = get_remote_control_point(); //获取遥控器值
	gimbal_init->gimbal_INT_angle_point = get_INS_angle_point(); //获取欧拉角数据
	gimbal_init->gimbal_INT_gyro_point = get_gyro_data_point();//获取角速度数据
	
	
	PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, mid_date.Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, mid_date.Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
	
		//初始化yaw电机pid
	gimbal_PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, 
		              mid_date.YAW_GYRO_ABSOLUTE_PID[0], mid_date.YAW_GYRO_ABSOLUTE_PID[1], mid_date.YAW_GYRO_ABSOLUTE_PID[2]);
	gimbal_PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, 
	                mid_date.PITCH_GYRO_ABSOLUTE_PID[0], mid_date.PITCH_GYRO_ABSOLUTE_PID[1], mid_date.PITCH_GYRO_ABSOLUTE_PID[2]);

	gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro_set ;//陀螺仪初始化

	kalmanCreate(&visual_x_fillter, 80, 300);  
	kalmanCreate(&visual_y_fillter, 80, 300); 
	Dt = DWT_GetDeltaT(&DWT_t) ;
	YawAutoLqrEso_Init(&gimbal_init->yaw_ctrl);
	YawAutoLqrEsoConfig_Init(&gimbal_init->yaw_cfg);
		
  
}


float gimbal_yaw_calc(void)//去零点
{
	
	float set;
	set = gimbal_ctrl.gimbal_yaw_motor.absolute_angle_set;
	float aset = (set +pi)/(2*pi) * 360;
	float aget = (gimbal_ctrl.gimbal_yaw_motor.absolute_angle+pi) / (2*pi) * 360;

	float res = 0.0f;
	if(aset >= aget) {
		if((aset - aget) > 180){	//aset = 350 aget = 10
			res = aset - 360 - aget;
		} else {					//aset = 20 aget = 10
			res = aset - aget;
		}
	} else {
		if((aset - aget) < -180){ 	//aset = 10 aget = 350
			res = 360 + aset - aget;
		} else {					//aset = 10 aget = 20
			res = aset - aget;
		}
	}
	
	return  (aget + res);  //角度环
}

fp32 ramp(fp32 input,fp32 frame_period)//斜波函数
{
	fp32 out;
	if(input>0)
	{
		out=input-frame_period;
	}
	else if(input<0)
	{
	  out=input+frame_period;
	}
  else if(input==0)
	{
	  out=0;
	}
	return out;
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //当前控制误差角度
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
			
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            //calculate max add_angle
            //计算出一个最大的添加角度，
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}


/** 
  * @brief         云台电机参数传出循环函数		          
  * @param[in]     gimbal整体云台参数结构体  
	* @author        刘根 
  * @retval        none 
  */
		static fp32 yaw_motor_visual_pixel = 0;
		static fp32 pitch_motor_visual_pixel = 0;
		static fp32 pitch_motor_visual_date = 0;
   	 fp32 pitch_angle_add = 0;
     fp32 yaw_angle_add = 0;

static uint8_t Visual_Ture_RInit_data_first;
static uint8_t Visual_Ture_RInit_data_end;
char i_count;
char switch_flag = 0;

void gimbal_loop(gimbal *loop)
{
	
	loop->gimbal_yaw_motor.offset_ecd = yaw_offset_ecd;
	loop->gimbal_pitch_motor.offset_ecd = pitch_offset_ecd;
	 		i_count++;
		if(i_count>50)
		{
			
			yaw_motor_visual_pixel = 0;
			pitch_motor_visual_pixel = 0;
			loop->visual_pitch_motor_speed.out=0;
			loop->visual_pitch_motor_angle.out=0;
		
		}
	float yaw_set;
	if(loop->yaw_first_angle_flag == 0 )//记录开机第一次计算出来的角度值
		 {
			 loop->gimbal_pitch_motor.absolute_angle_set = 0;
		   loop->yaw_first_angle_flag =1;
			 
		 }
		 
//	if(loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)  //锁死角度，吊射模式
//	{
//		 yaw_rc_multiple =   0.01f;
//    pitch_rc_multiple= 0.09f;
//		 YAW_MOUSE_SEN =   0.05f;
//     PITCH_MOUSE_SEN = 0.05f;
//		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET);
//		 gimbal_movement(&gimbal_ctrl);      //只计算云台遥感设置值	
//	}
		 
  if(loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)  //锁死角度，吊射模式
	{
		 yaw_rc_multiple =   0.00001f;
     pitch_rc_multiple = 0.000009f;
		 YAW_MOUSE_SEN =   0.0005f;
     PITCH_MOUSE_SEN = 0.0005f;
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET);
		 gimbal_movement(&gimbal_ctrl);      //只计算云台遥感设置值	
		
	}

		 
	if(loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO	) //自稳模式
	{
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET);
		 yaw_rc_multiple = 0.000025f;
     pitch_rc_multiple = 0.000015f;
     YAW_MOUSE_SEN = 0.001f;
     PITCH_MOUSE_SEN = 0.0003f;
		 gimbal_movement(&gimbal_ctrl);      //只计算云台遥感设置值

	}

	if(  loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_VISUAL)  //视觉模式
	{
   static int16_t yaw_channel = 0, pitch_channel = 0;	
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,RESET);


		if(1)
		{
     				
				rc_deadband_limit(loop->gimbal_rc->rc.ch[gimbal_yaw_CHANNEL], yaw_channel, 200);
				rc_deadband_limit(loop->gimbal_rc->rc.ch[gimbal_pitch_CHANNEL], pitch_channel, 200);

				loop->gimbal_yaw_motor.absolute_angle_set =-(loop->add_yaw);   //计算yaw轴目标角度值

			  loop->gimbal_pitch_motor.absolute_angle_set =(loop->add_pitch); //计算pitch轴目标角度值
								
				if(fabs(loop->gimbal_pitch_motor.absolute_angle_set-loop->gimbal_pitch_motor.absolute_angle)>0.4)//自瞄消抖
				{
					loop->gimbal_pitch_motor.absolute_angle_set = loop->gimbal_pitch_motor.absolute_angle;
				
				}
				if(fabs(loop->gimbal_yaw_motor.absolute_angle_set-loop->gimbal_yaw_motor.absolute_angle)>0.4)
				{
					loop->gimbal_yaw_motor.absolute_angle_set = loop->gimbal_yaw_motor.absolute_angle;
				
				}
				
			}
				
				loop->yaw_ref.theta_rad = loop->add_yaw;
				loop->yaw_ref.omega_rad_s = loop->add_yaw_v ;
				loop->yaw_ref.alpha_rad_s2 = loop->add_yaw_a;
				loop->yaw_feedback.theta_rad = loop->gimbal_yaw_motor.absolute_angle ;
				loop->yaw_feedback.omega_rad_s = loop->gimbal_yaw_motor.motor_gyro;
				loop->yaw_feedback.tau_meas_nm = loop->gimbal_yaw_motor.gimbal_motor_measure->given_current * 3 / 16384 * 0.741;
//				loop->yaw_feedback.feedback_ok = 0U;
				loop->yaw_cfg.eso_comp_enable = 1U;
				loop->yaw_cfg.eso_enable = 1U;
				loop->yaw_cfg.torque_slew_enable = 0U;
//				YawAutoLqrEso_Init(loop->yaw_ctrl);
//				YawAutoLqrEsoConfig_Init(loop->yaw_cfg);
				YawAutoLqrEso_Calc(&loop->yaw_ctrl,&loop->yaw_cfg,&loop->yaw_feedback,&loop->yaw_ref,Dt,&loop->yaw_output);
			gimbal_yaw_visual_tau_cmd = loop->yaw_output.tau_cmd_nm / 0.741 / 3 * 16384;
				gimbal_yaw_visual = 1;
				
			
				
			
			gimbal_movement(&gimbal_ctrl);   //计算absolute_angle_set值
		
		}
	else  {
		gimbal_yaw_visual = 0;
	}
	
	
	if(  loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_TEST_MODE)
	{
		current_m = loop->gimbal_yaw_motor.gimbal_motor_measure->given_current * (3.0f / 16384.0f);   //反馈电流值A
		
		real_hz = yaw_sweep.actual_frequency_hz;            //激励信号的频率
		w_m = loop->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm * ( 2 * pi / 60.0f);    //反馈角速度值rad/s
		
		if (loop->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_TEST_MODE )            //采样模式只会进行一次
        {
					if(gimbal_test_switch == 0 )
					{
						YawStepStart(YAW_STEP_DEFAULT_AMPLITUDE_A, 10U);                          //阶跃信号
					}
					else if(gimbal_test_switch == 1 )
					{
            YawSweepStart(YAW_SWEEP_DEFAULT_AMPLITUDE_A,YAW_SWEEP_CYCLES_PER_FREQ);     //激励信号
					}
					
        }
				
        loop->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_TEST_MODE ;
				if(gimbal_test_switch == 0 )
					{
						current_cmd = YawStepUpdate() * (16384.0f / 3.0f);    //阶跃信号电流目标值
					}
					else if(gimbal_test_switch == 1 )
					{
           current_cmd = (YawSweepUpdate() / 0.2f) * 16384 ;     //激励信号电流目标值
					}
//				CAN_cmd1_gimbal(current_cmd,0, 0, 0);
				current_set = current_cmd * (3.0f / 16384.0f);           //电流目标值A
				current_t = current_set * 0.741f;                        //扭矩目标值N*M

				if (yaw_sweep.running == 0U )
        {
//					current_cmd = 0;
					is_sweep = 0.0f;
        }
        else
        {
					is_sweep = 1.0f;
        }
				if (yaw_step.running == 0U )
				{
//					current_cmd = 0;
					is_step = 0.0f;
				}
				else
        {
          is_step = 1.0f;

        }
				if (yaw_sweep.running == 0U && yaw_step.running == 0U )
				{
					current_cmd = 0;
				}
		
	}
		yaw_set = gimbal_yaw_calc(); 
		
		//yaw
	  loop->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&loop->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 
																										((gimbal_ctrl.gimbal_yaw_motor.absolute_angle+pi) / (2*pi) * 360)*pi/180, 
																										yaw_set*pi/180, 
																										loop->gimbal_yaw_motor.motor_gyro);  
		
    loop->gimbal_yaw_motor.current_set = PID_calc(&loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, 
																					loop->gimbal_yaw_motor.motor_gyro, 
																					loop->gimbal_yaw_motor.motor_gyro_set);
		
		//pitch
		loop->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&loop->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, 
																										loop->gimbal_pitch_motor.absolute_angle, 
																										loop->gimbal_pitch_motor.absolute_angle_set, 
																										loop->gimbal_pitch_motor.motor_gyro);  
		
    loop->gimbal_pitch_motor.current_set = PID_calc(&loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, 
																					loop->gimbal_pitch_motor.motor_gyro, 
																					loop->gimbal_pitch_motor.motor_gyro_set);
		
	  if(loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_ZERO )//如果云台关闭则清除所有输出值
			{
				
				loop->yaw_motor_speed.out = 0;
				loop->pitch_motor_speed.out = 0;
				loop->gimbal_pitch_motor.absolute_angle_set = 0;
				loop->gimbal_yaw_motor.current_set = 0;
				loop->gimbal_pitch_motor.current_set = 0;	
				loop->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_ZERO ;		
				
			}

		if(gimbal_test_switch == 2){
			if(gimbal_yaw_visual == 0 )
			{
				CAN_cmd_gimbal(-loop->gimbal_yaw_motor.current_set, loop->gimbal_pitch_motor.current_set, 0, 0);
			}
			else
			{
				CAN_cmd_gimbal(-gimbal_yaw_visual_tau_cmd, loop->gimbal_pitch_motor.current_set, 0, 0);
			}
		}
		else{
			CAN_cmd1_gimbal(current_cmd,0, 0, 0);
		}
    

}

