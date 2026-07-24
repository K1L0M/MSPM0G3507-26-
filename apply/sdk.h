#ifndef __SDK_H
#define __SDK_H


typedef enum
{
	ROTATE=0,							//�ֶ�ƫ������ģʽ
  AZIMUTH=1,						//����ƫ���Ƕȿ���ģʽ
	CLOCKWISE=2,					//���ƫ���Ƕ�˳ʱ�����ģʽ	
	ANTI_CLOCKWISE=3,			//���ƫ���Ƕ���ʱ�����ģʽ	
	CLOCKWISE_TURN=4,			//���ٶȿ���˳ʱ��ģʽ
	ANTI_CLOCKWISE_TURN=5,//���ٶȿ�����ʱ��ģʽ
}YAW_CTRL_MODE;

typedef enum
{
	MOTOR1=0,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR_NUM
}motor;


typedef struct
{
	float throttle_control_output;	  	//���ſ��������������������δʹ��
	float roll_control_output;			  	//�����̬���������������������δʹ��
	float pitch_control_output;			  	//������̬���������������������δʹ��
	float yaw_control_output;				  	//ƫ����̬���������������������δʹ��
	float roll_outer_control_output;  	//�����̬����������
	float pitch_outer_control_output; 	//������̬����������
  float yaw_outer_control_output;	  	//ƫ����̬����������
  uint16_t motor_output[MOTOR_NUM];		//���ӳ�����ֵ��������δʹ��
  uint16_t temperature_control_output;//�¶ȿ��������ֵ��������δʹ��
	uint16_t yaw_ctrl_cnt;							//ƫ�����Ƽ�����
	YAW_CTRL_MODE yaw_ctrl_mode;				//ƫ������ģʽ
	uint8_t yaw_ctrl_start;							//ƫ�����ƿ�ʼ��־λ
	uint8_t yaw_ctrl_end;								//ƫ�����ƽ�����־λ
	uint32_t start_time_ms;							//ƫ�����ƿ�ʼʱ��
	uint32_t execution_time_ms;					//ƫ������ִ��ʱ��
	uint8_t init;												//ƫ�����Ƴ�ʼ����־λ
	//uint8_t yaw_least_cost_enable;      //ƫ��������С����ʹ�ܱ�־λ
	uint8_t roll_pitch_angle_limit_enable;
	LOCK_STATE unlock_flag;
}controller_output;


void trackless_params_init(void);
void ctrl_params_init(void);
void steer_control(float *output);
void steer_gyro_ctrl(void);
void steer_angle_ctrl(void);
void position_control(float fixed_threshold_cm,uint16_t feed_times);
void distance_control(void);	

void distance_control_with_speed_limit(float speed_limit);

extern float steer_gyro_scale;
extern float	steer_gyro_expect,steer_gyro_output,steer_angle_expect,steer_angle_error,steer_angle_output;

extern controller steerangle_ctrl,steergyro_ctrl;
extern controller distance_ctrl,azimuth_ctrl;
extern controller_output trackless_output;
#endif


