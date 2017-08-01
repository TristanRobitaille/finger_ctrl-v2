#include "Robot_IMU_Control.h"
// begin def
	int FREQ = 0;
	int FREQ_BASE = 0;
	int sensor_id = 0;	
	
	//DIRECTION - COMMANDES                                                                                  
	float CmdX_new = 0; float CmdY_new = 0;
	float CmdXout = 0;	float CmdYout = 0;
	float RightLeftComm			= 0.0;	                                                                        // -1 1
	float ForBackComm			= 0.0;	                                                                        // -1 1
	float UpDownComm			= 0.0;
	float RightLeftComm_ctrl	= 0.0;	                                                                        // -1 1
	float ForBackComm_ctrl		= 0.0;	                                                                        // -1 1
	float UpDownComm_ctrl		= 0.0;

	

	//Data 2D
	float x = 0;									//roll + offset
	float y = 0;									//pitch + offset
	//float x0 = 0;									//roll brute
	//float y0 = 0;									//pitch brute	
	float xoff = 0;									//offset - position neutre roll
	float yoff = 0;									//offset - position neutre pitch
	
	//Data Yaw
	/*float yaw = 0;
	float yawoff = 0;
	float XH = 0; float YH = 0; float yaw_mag = 0;
	float magx = 0; float magy = 0; float magz = 0;
	float gyrox = 0;
	float yaw_fused_sensor = 0;
	float yaw_fused_cpp = 0;*/

	float Amp = 0.0;
	float Theta = 0.0;
	float Orientation = 0.0;


	//CALIBRATION**************************************************************************************/
	int Valid = 1;									//indique la validité des marges et zones définies
	int DiagoActive = 1;							//0 = diagonale désactivée, 1 = diagonale activée
	float Theta_offset = 0;							//0 .. 360 degrés, mapping
	
	float Amin_BACK = 1;
	float Amax_BACK = 20;
	float Amin_D = 3;
	float Amax_D = 50;
	float Amax = 50;								//inclinaison maximale
	float Amin = 3;									//seuil d'inclinaison
	float ZoneForw = 90;	float DeltaForw = 20;	//Angle Forward	, Marge Forwad
	float ZoneRight = 0;	float DeltaRight = 30;	//Angle Right	, Marge Right
	float ZoneLeft = 180;	float DeltaLeft = 30;	//Angle Left	, Marge Left
	float ZoneBack = 270;	float DeltaBack = 20;	//Angle Backward, Marge Backward

	float MaxUp = 20;		float DeltaMaxUp = 10;	//Yaw Angle Right (Move Up)	, Marge
	float MaxDown = -20;	float DeltaMaxDown = 10;//Yaw Angle Left (Mode Down), Marge
	float NeutralUpDown = 0; float DeltaNeutralUpDown = 10;
	/**************************************************************************************************/

	//BUTTONS                                                                                                   
	int PRESSED = 0;	
	int pressedtime = 0;
	int B_INDEX = 0;																						
	bool HOME_PRESSED = 0;
	bool BACK = 0;

//end def
const float Robot_IMU_Control::num_lp_10Hz[3] = { 0.355, 0.355, 0.0 };
const float Robot_IMU_Control::num_lp_25Hz[3] = { 0.755, 0.755, 0.0 };
const float Robot_IMU_Control::den_lp_10Hz[3] = { 1.0, -0.2905, 0.0 };
const float Robot_IMU_Control::den_lp_25Hz[3] = { 1.0, 0.5095, 0.0 };

const double Robot_IMU_Control::comp_headset[3][3] = { { 0.000168679716274983, -5.17710741425967e-06, -8.77884394210979e-07 },
													   { -5.17710741425967e-06, 0.000159571222440490, -3.27611632378113e-06 },
													   { -8.77884394210986e-07, -3.27611632378113e-06, 0.000166456977421752 } };

const double Robot_IMU_Control::hardiron_headset[3] = { -725.100317519760, 491.703440889862, -53.3108814506022 };