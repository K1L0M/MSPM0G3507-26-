#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"
#include "debug_if.h"

int16_t sdk_work_mode=0;


#define wheel_space_cm  12.8f//轮间距  12.8cm



void sdk_duty_run(void)
{
	if(trackless_output.init==0)
	{		
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output=0;
		trackless_output.init=1;
		flight_subtask_reset();//复位sdk子任务状态量
	}
	if(smartcar_imu.imu_convergence_flag!=1) return;//姿态解算系统就位
	
	switch(sdk_work_mode)
	{
		case -10://初始调试模式，用于确定电机运动方向时使用
		{
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=motion_test_pwm_default;//默认输出百分之50占空的pwm
		}
		break;
		case -9://初始调试模式，用于确定舵机中值时使用
		{
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=0;//默认输出百分之50占空的pwm
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		}
		break;
		case -2://调速测试模式——速度期望来源于遥控
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case -1://调速测试模式——速度期望来源于按键设定
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case 0://遥控控制
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;		
		case 1://基于灰度管的自主寻迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;		
		case 2://顺时针转动90°
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
			flight_subtask_1();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 3://逆时针转动90°
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
			flight_subtask_1();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 4://顺时针转动90°
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
			flight_subtask_3();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 5://以30deg/s的角速度逆时针转动3000ms
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			flight_subtask_4();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 6://基于超声波测距的前向避撞小车
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			flight_subtask_5();
			steer_control(&turn_ctrl_pwm);	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		}
		break;		
		case 7://两轮平衡车
		{
			speed_ctrl_mode=2;//速度控制方式为平衡速度控制
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			speed_setup=remote_data_remap(&RC_Data ,RC_PITCH_CHANNEL ,50,50,false);//将遥杆动作位映射成期望速度
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
			
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定
			steer_control(&turn_ctrl_pwm);	//转向陀螺仪控制
			balance_control_single_control();			
		}
		break;
		case 8://基于两轮差速模型的速度、角速度控制,用于机载计算机ROS端发生运动指令控制下位机差速平台
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
	
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定
			turn_ctrl_pwm=trackless_output.yaw_outer_control_output*DEG2RAD;//期望角速度转换成弧度制
			
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*wheel_space_cm*0.5f;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*wheel_space_cm*0.5f;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 9://地面站航点控制模式，通过无名创新地面站V1.0.6版本发布航点
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control(3.0f,10);
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);			
		}
		break;		
		case 10://OPENMV视觉自主寻迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 11://双电机+前轮舵机转向遥控控制
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+RC_Data.rcdata[RC_YAW_CHANNEL]-1500);	
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}	
		break;
		case 12://双电机+前轮舵机转向，视觉自主寻迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			vision_turn_control_50hz(&turn_ctrl_pwm);//基于OPENMV视觉处理的转向控制
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
			//期望速度
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 13://倒车入库
		{
			auto_reverse_stall_park();
		}
		break;
		case 14://侧方停车
		{
			auto_parallel_park();
		}		
		break;
		case 15://2022年7月份省赛小车跟随行驶系统赛道,内外圈交替循迹
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 16://2024年电赛H题第1问
		{
			auto_drive_smartcar_duty1();
		}
		break;
		case 17://2024年电赛H题第2问
		{
			auto_drive_smartcar_duty2();
		}
		break;
		case 18://2024年电赛H题第3问
		{
			auto_drive_smartcar_duty3(1);
		}
		break;
		case 19://2024年电赛H题第4问
		{
			auto_drive_smartcar_duty4(4);
		}
		break;
		case 20://2024年电赛H题发挥部分
		{
			auto_nav_point(1);
		}
		break;
		case 21://直接偏航控制-原地掉头学习案例
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制			
			flight_subtask_yaw_angle_ctrl(180);
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 22://角速度内环调试（目标rate=输入）
		{
			static int16_t prev_mode = -1;

			if (prev_mode != 22)
			{
				pid_integrate_reset(&steergyro_ctrl);
				prev_mode = 22;
			}

			speed_ctrl_mode = 1;

			// 外环输出直接作为内环期望角速度（绝对物理量）
			steer_gyro_expect = trackless_output.yaw_outer_control_output;
			steer_gyro_ctrl();

			turn_ctrl_pwm = -steergyro_ctrl.output;

			speed_setup = 0;
			speed_expect[0] =  turn_ctrl_pwm * steer_gyro_scale;
			speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale;
			speed_control_100hz(speed_ctrl_mode);
		}
		break;
		case 23://角度-角速度串级调试（目标yaw=输入）
		{
			static int16_t prev_mode = -1;

			if (prev_mode != 23)
			{
				pid_integrate_reset(&steergyro_ctrl);
				pid_integrate_reset(&steerangle_ctrl);
				prev_mode = 23;
			}

			speed_ctrl_mode = 1;

			// 外环：角度P控制
			steer_angle_expect = trackless_output.yaw_outer_control_output;
			steer_angle_ctrl();

			// 外环输出限幅后作为内环期望角速度
			float angle_output = steer_angle_output;
			if (angle_output > 100.0f)  angle_output = 100.0f;
			if (angle_output < -100.0f) angle_output = -100.0f;

			// 内环：角速度PID
			steer_gyro_expect = angle_output;
			steer_gyro_ctrl();

			turn_ctrl_pwm = -steergyro_ctrl.output;

			speed_setup = 0;
			speed_expect[0] =  turn_ctrl_pwm * steer_gyro_scale;
			speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale;
			speed_control_100hz(speed_ctrl_mode);
		}
		break;
		case 24://相对角度转向,与mode2同链路
		{
			static float turn_target = 0;
			static float turn_accum = 0;
			static float turn_prev_yaw = 0;
			static float turn_last_cmd = 0;
			static uint8_t turn_running = 0;
			static int16_t prev_mode = -1;

			if (prev_mode != 24)
			{
				float input = trackless_output.yaw_outer_control_output;
				turn_target = input;
				turn_last_cmd = input;
				turn_accum = 0;
				turn_prev_yaw = smartcar_imu.rpy_deg[_YAW];
				turn_running = (fabsf(input) > 0.5f) ? 1 : 0;
				pid_integrate_reset(&steergyro_ctrl);
				pid_integrate_reset(&steerangle_ctrl);
				prev_mode = 24;
			}
			else if (!turn_running)
			{
				float input = trackless_output.yaw_outer_control_output;
				if (fabsf(input - turn_last_cmd) > 0.1f && fabsf(input) > 0.5f)
				{
					turn_target = input;
					turn_last_cmd = input;
					turn_accum = 0;
					turn_prev_yaw = smartcar_imu.rpy_deg[_YAW];
					turn_running = 1;
					pid_integrate_reset(&steergyro_ctrl);
					pid_integrate_reset(&steerangle_ctrl);
				}
			}

			speed_ctrl_mode = 1;

			if (turn_running)
			{
				float curr = smartcar_imu.rpy_deg[_YAW];
				float dyaw = curr - turn_prev_yaw;
				if (dyaw > 180) dyaw -= 360;
				if (dyaw < -180) dyaw += 360;
				turn_prev_yaw = curr;

				if (turn_target > 0)
					turn_accum += -dyaw;  // CW
				else
					turn_accum += dyaw;   // CCW

				float remaining = fabsf(turn_target) - turn_accum;
				if (remaining < 3.0f || remaining < 0)
				{
					turn_running = 0;
					turn_ctrl_pwm = 0;
				}
				else
				{
					// same chain as mode2 CLOCKWISE:
					// steer_angle_ctrl() -> steer_gyro_ctrl()
					float chunk = (remaining > 175.0f) ? 175.0f : remaining;
					steer_angle_expect = (turn_target > 0)
					                   ? (curr - chunk)
					                   : (curr + chunk);
					steer_angle_ctrl();
					steer_gyro_expect = steer_angle_output;
					steer_gyro_ctrl();
					turn_ctrl_pwm = -steergyro_ctrl.output;
				}
			}
			else
			{
				turn_ctrl_pwm = 0;
			}

			speed_setup = 0;
			speed_expect[0] =  turn_ctrl_pwm * steer_gyro_scale;
			speed_expect[1] = -turn_ctrl_pwm * steer_gyro_scale;
			speed_control_100hz(speed_ctrl_mode);
		}
		break;
		case 50://陀螺仪角速度内环调试模式(理想rate=输入)
		{
			speed_ctrl_mode=1;//速度控制方式为期望值速度控制
			// 直接设置角速度期望并调用内环，跳过 steer_control 的 ROTATE 混合逻辑
			// ROTATE 在 yaw_outer_control_output==0 时会切入角度锁定，违背纯内环调试初衷
			steer_gyro_expect=trackless_output.yaw_outer_control_output;
			steer_gyro_ctrl();
			turn_ctrl_pwm=-steer_gyro_output;//保持与 steer_control 相同的输出极性
			speed_setup=0;//原地旋转无速度分量
			//差速赋值
			speed_expect[0]=turn_ctrl_pwm*steer_gyro_scale;//左轮期望速度
			speed_expect[1]=-turn_ctrl_pwm*steer_gyro_scale;//右轮期望速度
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		}
		break;
		case 25: // Odom round-trip: RET=record start/return, RST=reset
		{
			static uint8_t  state = 0;
			static float    start_x = 0, start_y = 0;
			static int16_t  prev_mode = -1;
			if (prev_mode != 25) {
				state     = 0;
				prev_mode = 25;
								g_odo_return_trigger = 0;
				g_odo_reset_trigger  = 0;
				pid_integrate_reset(&steergyro_ctrl);
				pid_integrate_reset(&steerangle_ctrl);
				pid_integrate_reset(&azimuth_ctrl);
				pid_integrate_reset(&distance_ctrl);
			}
			switch (state) {
				case 0: // WAIT - wait for RET to record start
					speed_ctrl_mode = 0;
					motion_ctrl_pwm = 0;
					speed_control_100hz(speed_ctrl_mode);
					trackless_output.unlock_flag = UNLOCK;
					if (g_odo_return_trigger) {
						g_odo_return_trigger = 0;
						start_x = smartcar_imu.state_estimation.pos.x;
						start_y = smartcar_imu.state_estimation.pos.y;
						ngs_nav_ctrl.ctrl_finish_flag = 0;
						state = 1;
					}
					break;
				case 1: // PUSH - manual push phase
					speed_ctrl_mode = 0;
					motion_ctrl_pwm = 0;
					speed_control_100hz(speed_ctrl_mode);
					trackless_output.unlock_flag = UNLOCK;
					if (g_odo_return_trigger) {
						g_odo_return_trigger = 0;
						ngs_nav_ctrl.x = start_x; ngs_nav_ctrl.y = start_y;
						ngs_nav_ctrl.update_flag = 1;
						state = 2;
					}
					if (g_odo_reset_trigger) { g_odo_reset_trigger = 0; state = 0; }
					break;
				case 2: // RETURN - auto return
					position_control(3.0f, 5);
					turn_ctrl_pwm = steer_gyro_output;
					speed_setup   = distance_ctrl.output;
					speed_expect[0] = speed_setup - turn_ctrl_pwm * steer_gyro_scale;
					speed_expect[1] = speed_setup + turn_ctrl_pwm * steer_gyro_scale;
					speed_ctrl_mode = 1;
					speed_control_100hz(speed_ctrl_mode);
					trackless_output.unlock_flag = UNLOCK;
					if (ngs_nav_ctrl.ctrl_finish_flag == 1) {
						float err_x = smartcar_imu.state_estimation.pos.x - start_x;
						float err_y = smartcar_imu.state_estimation.pos.y - start_y;
						float err_d = sqrtf(err_x * err_x + err_y * err_y);
						char buf[64];
						snprintf(buf, sizeof(buf),
							"ODO_DONE dX=%.2f dY=%.2f Err=%.2f\r\n",
							err_x, err_y, err_d);
						DebugIF_Print(buf);
						state = 3;
					}
					break;
				default: // DONE - stop, wait for RST
					speed_ctrl_mode = 0;
					motion_ctrl_pwm = 0;
					speed_control_100hz(speed_ctrl_mode);
					if (g_odo_reset_trigger) { g_odo_reset_trigger = 0; state = 0; }
					break;
			}
		}
		break;
		default:
		{
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			trackless_output.yaw_ctrl_mode=ROTATE;//偏航控制模式
			//trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//偏航期望来源于横滚杆给定		
			steer_control(&turn_ctrl_pwm);
			//speed_setup=RC_Data.rc_rpyt[RC_PITCH];//速度期望来源于俯仰杆给定	
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);			
		}
	}
}
