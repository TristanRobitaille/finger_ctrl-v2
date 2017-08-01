#ifndef _ROBOT_IMU_CONTROL_H
#define _ROBOT_IMU_CONTROL_H


#pragma once
//#include "KinovaTypes.h"
#include "Defined_Macro.h"
#include <string>
//#include <Windows.h>
#include <math.h>
#include <stdlib.h>
//using namespace System;
//using namespace std;

namespace KinDrv{
class Robot_IMU_Control
{
public :
	int FREQ ;
	int FREQ_BASE ;
	int sensor_id;//	
	
	//DIRECTION - COMMANDES                                                                                  
	float CmdX_new ; float CmdY_new;
	float CmdXout;	float CmdYout ;
	float RightLeftComm	;	                                                                        // -1 1
	float ForBackComm	;	                                                                        // -1 1
	float UpDownComm	;
	float RightLeftComm_ctrl;	                                                                        // -1 1
	float ForBackComm_ctrl	;	                                                                        // -1 1
	float UpDownComm_ctrl	;

	//10Hz filter
	static const double num_lp_10Hz[3];
	static const double num_lp_25Hz[3];
	static const double den_lp_10Hz[3];
	static const double den_lp_25Hz[3];

	//Calib
	static const double comp_headset[3][3];
	static const double hardiron_headset[3];

	//RAW DATA
	float acc[3];
	float gyr[3];
	float mag[3];
	float pitch[3]; 
	float x_pitch[3];
	float roll[3];  
	float x_roll[3];
	float yaw[3];   
	float x_yaw[3];

	//Data 2D
	float x ;									//roll + offset
	float y ;									//pitch + offset
	float x0 ;									//roll brute
	float y0 ;									//pitch brute	
	float xoff ;									//offset - position neutre roll
	float yoff ;									//offset - position neutre pitch
	
	//Data Yaw
	/*float yaw ;
	float yawoff ;
	float XH ; float YH ; float yaw_mag ;
	float magx ; float magy ; float magz;
	float gyrox ;
	float yaw_fused_sensor ;
	float yaw_fused_cpp ;*/

	float Amp;
	float Theta ;
	float Orientation;

	//CALIBRATION**************************************************************************************/
	int Valid;									//indique la validité des marges et zones définies
	int DiagoActive ;							//0 = diagonale désactivée, 1 = diagonale activée
	float Theta_offset ;							//0 .. 360 degrés, mapping
	
	float Amin_BACK ;
	float Amax_BACK ;
	float Amin_D ;
	float Amax_D ;
	float Amax ;								//inclinaison maximale
	float Amin ;									//seuil d'inclinaison
	float ZoneForw ;	float DeltaForw ;	//Angle Forward	, Marge Forwad
	float ZoneRight ;	float DeltaRight ;	//Angle Right	, Marge Right
	float ZoneLeft ;	float DeltaLeft ;	//Angle Left	, Marge Left
	float ZoneBack ;	float DeltaBack ;	//Angle Backward, Marge Backward

	float MaxUp ;		float DeltaMaxUp ;	//Yaw Angle Right (Move Up)	, Marge
	float MaxDown ;	float DeltaMaxDown;//Yaw Angle Left (Mode Down), Marge
	float NeutralUpDown ; float DeltaNeutralUpDown ;
	/**************************************************************************************************/

	//BUTTONS                                                                                                   
	int PRESSED;	
	int pressedtime;
	int B_INDEX ;																						
	bool HOME_PRESSED ;
	bool BACK ;
	Robot_IMU_Control();

	~Robot_IMU_Control();

	void read_payload (int data[]);
	
	void sensor_fusion();

	void algo();

	float C2_2(int var);
	

	void wrap_angle(float * p_angle);
	

	float rad2deg(float angle_rad);
	

	float deg2rad(float angle_deg);
	
};
}
#endif
