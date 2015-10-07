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
//Foobar is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with MinikDCS.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////


#include "CameraVision.h"

CameraVision::CameraVision(int camera_type, bool load_cam, char* PORT)
{
	if(load_cam)
	{
		port = PORT;
		this->camera_type = camera_type;
		setupConn();
	}
}

void CameraVision::setupConn()
{
	if(camera_type == CMU_CAM)
	{
#ifdef __linux__

		int serial_flags = O_RDWR | O_NOCTTY | O_NDELAY;
		camComm = open(port, serial_flags);
		if( camComm == -1)
		{
			cout << "Error connecting CMU Cam" << endl;
		}
		fcntl(camComm, F_SETFL, FNDELAY);
		
		struct termios port_settings;
		tcgetattr(camComm, &port_settings);
		
		cfsetispeed(&port_settings, B115200);
		cfsetospeed(&port_settings, B115200);
		
		
		port_settings.c_cflag |= (CLOCAL | CREAD);
		port_settings.c_cflag |= CS8;
		port_settings.c_cflag &= ~CRTSCTS;
		port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
		tcsetattr(camComm, TCSANOW, &port_settings);
		usleep(500);
		
#elif defined(_WIN32)

		camComm = new CSerial();
		camComm->Open(_T(serialPORT),0,0,false);	//Specify which port will be used
		camComm->Setup(CSerial::EBaud115200,CSerial::EData8,CSerial::EParNone,CSerial::EStop1);
		Sleep(500);
#endif		

		
	}
	else if(camera_type == SURVEYOR_CAM)
	{
#ifdef __linux__

#elif defined(_WIN32)
		srvClient = new ClientSocket( surveyorIP, 10001 );
#endif	
	}
}

void CameraVision::closeConn()
{	
#ifdef __linux__
	if(camera_type == CMU_CAM)
		close(camComm);
	else if(camera_type == SURVEYOR_CAM)
	{
		
	}
#elif defined(_WIN32)
	if(cameraType == CMU_CAM)
		camComm->Close();
	else if(cameraType == SURVEYOR_CAM)
		srvClient->Close();
#endif	
}

bool CameraVision::grabFrame(Mat grabbed_image)
{
	
#ifdef __linux__
	if(camera_type == CMU_CAM)
	{
		//Camera uses JPEG compression,
		//total number of elements will be less
		//than that.
		char img_buffer[ CMU_CAM_W * CMU_CAM_H *3]; 
		
		unsigned char cmd[] = {'S', 'J', '\r','\n'};
		write( camComm, cmd, sizeof(cmd) -1 );
	
		usleep(100);

		int bytesRead;

		char waitStr[5];
		char dummyChar;
		read(camComm,&waitStr[0],sizeof(char));
		read(camComm,&waitStr[1],sizeof(char));
		read(camComm,&waitStr[2],sizeof(char));
		while(true)
		{
			if(waitStr[0] == 'A' && waitStr[1] == 'C' && waitStr[2] == 'K')
				break;
			bytesRead = read(camComm,&dummyChar,sizeof(char));
			if(bytesRead > 0 )
			{
				waitStr[0] = waitStr[1];
				waitStr[1] = waitStr[2];
				waitStr[2] = dummyChar;
			}
		}
		bytesRead = read(camComm,&dummyChar,sizeof(char));

		int i = 0;
		while(true)
		{
			if(waitStr[0] == 'J' && waitStr[1] == 'P' && waitStr[2] == 'G')
				break;
			bytesRead = read(camComm,&dummyChar,sizeof(char));
			if(bytesRead > 0 )
			{
				waitStr[0] = waitStr[1];
				waitStr[1] = waitStr[2];
				waitStr[2] = dummyChar;
				img_buffer[i++] = dummyChar;
			}

		}
		cv::Mat imgbuf(cv::Size(CMU_CAM_W,CMU_CAM_H), CV_8UC3,img_buffer);
		cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
		
		#ifdef SHOW_GRABBED_IMG
			cv::imshow("Grabbed Image",img);
			cv::waitKey(0);
		#endif
		
		grabbed_image = img;
		
		return 1;
	}
	else if(camera_type == SURVEYOR_CAM)
	{
		
	}
	
#elif defined(_WIN32)

	if(cameraType == CMU_CAM)
	{
		//Camera uses JPEG compression,
		//total number of elements will be less
		//than that.
		char img_buffer[ CMU_CAM_W * CMU_CAM_H *3]; 
		
		char tempPack[3];
		tempPack[0] = 'S';
		tempPack[1] = 'J';
		tempPack[2] = '\r';
		camComm->Write(tempPack, 3*sizeof(char));
		Sleep(300);

		char BUFFER[100000];
		DWORD dwBytesRead;

		char waitStr[5];
		char dummyChar;
		int trigger = 0;
		camComm->Read(&waitStr[0],sizeof(char),&dwBytesRead);
		camComm->Read(&waitStr[1],sizeof(char),&dwBytesRead);
		camComm->Read(&waitStr[2],sizeof(char),&dwBytesRead);
		while(true)
		{
			if(waitStr[0] == 'A' && waitStr[1] == 'C' && waitStr[2] == 'K')
				break;
			camComm->Read(&dummyChar,sizeof(char),&dwBytesRead);
			if(dwBytesRead > 0 )
			{
				waitStr[0] = waitStr[1];
				waitStr[1] = waitStr[2];
				waitStr[2] = dummyChar;
			}
		}
		camComm->Read(&dummyChar,sizeof(char),&dwBytesRead);

		while(true)
		{
			if(waitStr[0] == 'J' && waitStr[1] == 'P' && waitStr[2] == 'G')
				break;
			camComm->Read(&dummyChar,sizeof(char),&dwBytesRead);
			if(dwBytesRead > 0 )
			{
				waitStr[0] = waitStr[1];
				waitStr[1] = waitStr[2];
				waitStr[2] = dummyChar;
				
				img_buffer[i++] = dummyChar;
			}
		}
		
		cv::Mat imgbuf(cv::Size(CMU_CAM_W,CMU_CAM_H), CV_8UC3,img_buffer);
		cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
		
		#ifdef SHOW_GRABBED_IMG
			cv::imshow("Grabbed Image",img);
			cv::waitKey(0);
		#endif
	}

	else if(cameraType == SURVEYOR_CAM)
	{
		char recvBuf[500];
		srvClient->Send( "I" );

		srvClient->Receive(10, recvBuf);

		int count  = 0;

		do
		{
			if(srvClient->isEmpty() == 1)
				break;

			count = srvClient->Receive(50, recvBuf);

			if (count > 0)
				out.write(recvBuf, count);
			else
				printf("recv failed: %d\n", WSAGetLastError());

		}while(count > 0);

		out.close();
	}
#endif	
	return 0;
}

