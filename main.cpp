#include "CameraVision.h"

int main()
{
	Mat img;
	CameraVision* cmu_cam = new CameraVision(CMU_CAM,LOAD_CAM,"/dev/ttyUSB0");
	
	while(1)
		cmu_cam->grabFrame(img);
	
	return 0;
}
