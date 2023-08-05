#ifndef _CONFIG_H
#define _CONFIG_H

#define PI2RAD 57.2957795f

#define X_OFFSET 129.5f //pitch轴画布原点
#define Y_OFFSET 132.0f//yaw轴画布原点

//#define PITCH_ZERO  0x05D0
#define PITCH_ZERO_ANGLE  130.0f //pitch世界水平原点

//相机像素
#define CAMERA_X 85.0f
#define CAMERA_Y 85.0f

#define CANVAS_X 50.0f
#define CANVAS_Y 50.0f

#define CAM2CAN_X CANVAS_X/CAMERA_X
#define CAM2CAN_Y CANVAS_Y/CAMERA_Y


#define CAMERA_X_GAIN 1.14f
#define CAMERA_Y_GAIN 1.04f
#define CAMERA_Y_OFFSET -0.4f
#define CAMERA_X_OFFSET 0.4f
#endif
