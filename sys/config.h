#ifndef _CONFIG_H
#define _CONFIG_H

#define PI2RAD 57.2957795f

#define X_OFFSET 129.5f //pitch�ử��ԭ��
#define Y_OFFSET 132.0f//yaw�ử��ԭ��

//#define PITCH_ZERO  0x05D0
#define PITCH_ZERO_ANGLE  130.0f //pitch����ˮƽԭ��

//�������
#define CAMERA_X 85.0f
#define CAMERA_Y 85.0f

#define CANVAS_X 50.0f
#define CANVAS_Y 50.0f

#define CAM2CAN_X CANVAS_X/CAMERA_X
#define CAM2CAN_Y CANVAS_Y/CAMERA_Y


#define CAMERA_X_GAIN 1.14f
#define CAMERA_Y_GAIN 1.09f
#define CAMERA_Y_OFFSET 0.0f
#define CAMERA_X_OFFSET 0.7f
#endif
