#ifndef __VISION_H
#define __VISION_H

#define SDK_Target_Length  53//45

typedef enum 
{
  UART7_SDK=0,
	UART1_SDK,
}COM_SDK;


typedef union
{
	gray_flags gray;
	uint16_t value;
}split_state;

typedef struct
{
  uint16_t x;
  uint16_t y; 
  uint16_t pixel;  
	uint8_t flag;
	uint8_t state;		
	int16_t angle;
	uint16_t distance;
	uint16_t apriltag_id;
	uint16_t width;
	uint16_t height;
	uint8_t fps;
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t reserved4;
	uint16_t range_sensor1;
	uint16_t range_sensor2;
	uint16_t range_sensor3;
	uint16_t range_sensor4;
	uint8_t camera_id;
	int32_t reserved1_int32;
	int32_t reserved2_int32;
	int32_t reserved3_int32;
	int32_t reserved4_int32;
	//
	uint8_t sdk_mode;
	float x_cm;
	float y_cm;
	float z_cm;	
	float x_pixel_size;
	float y_pixel_size;
	float z_pixel_size;
	float apriltag_distance;
	uint16_t trust_cnt;
	uint16_t trust_flag;
	uint8_t line_ctrl_enable;
	uint8_t target_ctrl_enable;
	vector3f sdk_target,sdk_target_offset;
	float sdk_angle;
	split_state x0,y0;
	uint8_t carpark_type;
	uint8_t carpark_num;
	uint8_t carpark_flag[2][4];
}Target_Check;//Ä¿±ê¼ì²â

void SDK_Data_Receive_Prepare_1(uint8_t data);
void SDK_Data_Receive_Prepare_2(uint8_t data);

extern Target_Check camera1,camera2;



#define MAX_DETECTION_TARGETS 10

typedef struct {
    uint8_t  ball_id;
    float    conf;
    float    dist;
    uint16_t cx;
} DetectionTarget;

typedef struct {
    uint8_t         num_targets;
    DetectionTarget targets[MAX_DETECTION_TARGETS];
    volatile uint8_t frame_ready;
    uint32_t        frame_count;
    uint32_t        empty_frame_count;
} DetectionFrame;

extern DetectionFrame g_detection;
void Detection_Data_Receive(uint8_t ch);
void Detection_ProcessFrame(void);
#endif

