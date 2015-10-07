//////////////////////////////////////////////////////////////////////
//Author: Mahmut Demir
//E-mail: mahmutdemir@gmail.com
//Intelligent Systems Laboratory, Bogazici University, 2012
//
//This file is part of MinikDCS(Development and Control Software)
//It contains required functions used in the control of differential
//wheeled robots.
//
//MinikDCS is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//MinikDCS is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with MinikDCS.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CameraVision
#define _CameraVision

//Type of camera used in robots
#define CMU_CAM 1
#define SURVEYOR_CAM 2

#define CMU_CAM_W 88
#define CMU_CAM_H 143

#define LOAD_CAM 1
#define NOT_LOAD_CAM 0

#define SHOW_GRABBED_IMG 1




#ifdef __linux__
	#include <stdlib.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <termios.h>
#elif defined(_WIN32)
	#include <tchar.h>
	#include <windows.h>
	#include "Serial.h"
	#include "imgGrab/ClientSocket.h"
#endif

using namespace cv;
using namespace std;


//Contains various methods and functions used
//processing vision data taken from cameras
class CameraVision
{
public:	
	CameraVision(int camera_type, bool load_cam, char* PORT); //type of camera, load camera or not, port of the camera
	void setupConn();
	bool grabFrame(Mat grabbed_image);
	void closeConn();

private:
#ifdef __linux__
	int camComm;
#elif defined(_WIN32)
	CSerial *camComm;
	ClientSocket *srvClient;
#endif

public:	
	char* port;
	int camera_type;

};
#endif
