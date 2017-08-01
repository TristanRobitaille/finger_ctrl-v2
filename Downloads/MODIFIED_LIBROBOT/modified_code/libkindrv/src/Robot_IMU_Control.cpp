#include "Robot_IMU_Control.h"

#define HEADSET 10
namespace KinDrv{
	/* ########################## ALL PARAMS #####################*/
	int FREQ = 0;
	int FREQ_BASE = 0;
	int sensor_id = 0;//	
	
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
	float yoff = 0;	 //offset - position neutre pitch

	float acc[3];
	float gyr[3];
	float mag[3];
	float pitch[3]; 
	float x_pitch[3];
	float roll[3];  
	float x_roll[3];
	float yaw[3];   
	float x_yaw[3];								
	
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
	
	

const double Robot_IMU_Control::num_lp_10Hz[3] = { 0.355, 0.355, 0.0 };
const double Robot_IMU_Control::num_lp_25Hz[3] = { 0.755, 0.755, 0.0 };
const double Robot_IMU_Control::den_lp_10Hz[3] = { 1.0, -0.2905, 0.0 };
const double Robot_IMU_Control::den_lp_25Hz[3] = { 1.0, 0.5095, 0.0 };
const double Robot_IMU_Control::comp_headset[3][3] = { { 0.000168679716274983, -5.17710741425967e-06, -8.77884394210979e-07 },
													   { -5.17710741425967e-06, 0.000159571222440490, -3.27611632378113e-06 },
													   { -8.77884394210986e-07, -3.27611632378113e-06, 0.000166456977421752 } };

const double Robot_IMU_Control::hardiron_headset[3] = { -725.100317519760, 491.703440889862, -53.3108814506022 };


	/* ##### METHODS ##### */
Robot_IMU_Control::Robot_IMU_Control(void){
	}

Robot_IMU_Control::~Robot_IMU_Control(void){
	}
	
void Robot_IMU_Control::read_payload ( int data[])
{
		if (data[1] != 0)
		{
			FREQ = 1000 / data[1];											// MSP430 Sensor Acquisition Frequency
		}
		sensor_id = data[29];
		if (sensor_id == HEADSET)
		{
			acc[0] = C2_2((data[14] << 8) + data[15]);
			acc[1] = C2_2((data[16] << 8) + data[17]);
			acc[2] = C2_2((data[18] << 8) + data[19]);

			gyr[0] = C2_2((data[8] << 8) + data[9]);
			gyr[1] = C2_2((data[10] << 8) + data[11]);
			gyr[2] = C2_2((data[12] << 8) + data[13]);

			mag[0] = C2_2((data[2] << 8) + data[3]);
			mag[1] = C2_2((data[4] << 8) + data[5]);
			mag[2] = C2_2((data[6] << 8) + data[7]);
		}
}


/*  ________ */


void Robot_IMU_Control::sensor_fusion()
	{
		if (sensor_id == HEADSET)
		{
			//pitch
			pitch[0] = ALPHA_ACC*rad2deg(atan2(-acc[1], -acc[0])) + ALPHA_GYRO*(pitch[0] + (-gyr[2]) * GAIN_GYRO*(1 / FREQ_GYRO));
			x_pitch[0] = pitch[0];
			pitch[1] = num_lp_10Hz[0] * x_pitch[0] + num_lp_10Hz[1] * x_pitch[1] - den_lp_10Hz[1] * pitch[2];
			x_pitch[1] = x_pitch[0];
			pitch[2] = pitch[1];

			//roll
			roll[0] = ALPHA_ACC*rad2deg(atan2(-acc[2], -acc[0])) + ALPHA_GYRO*(roll[0] + gyr[1] * GAIN_GYRO*(1 / FREQ_GYRO));
			x_roll[0] = roll[0];
			roll[1] = num_lp_10Hz[0] * x_roll[0] + num_lp_10Hz[1] * x_roll[1] - den_lp_10Hz[1] * roll[2];
			x_roll[1] = x_roll[0];
			roll[2] = roll[1];

			//yaw 
		}
	}
	/* ________*/ 
	
void Robot_IMU_Control::algo()
{
		Amin_D = Amin;
		Amax_D = Amax;
		BACK = 0;
		
		x0 = roll[2];
		y0 = pitch[2];

		y = y0 - yoff;
		x = x0 - xoff;
		
		if (ZoneRight > 180)
		{
			ZoneRight = ZoneRight - 180;
		}
		float Ri1 = ZoneRight - DeltaRight / 2;
		float Ri2 = Ri1 + 360;

		if ((ZoneRight + DeltaRight / 2) > (ZoneForw - DeltaForw / 2) || (ZoneForw + DeltaForw / 2) > (ZoneLeft - DeltaLeft / 2) || (ZoneLeft + DeltaLeft / 2) > (ZoneBack - DeltaBack / 2) || (ZoneBack + DeltaBack / 2) > (Ri2))
		{
			Valid = 0;
			ZoneForw = 90;
			ZoneRight = 10;
			ZoneLeft = 170;
			ZoneBack = 270;
			DeltaForw = 20;
			DeltaRight = 60;
			DeltaLeft = 60;
			DeltaBack = 20;
		}

		Amp = sqrt(x*x + y*y);
		Theta = atan2(y, x) * 180 / M_PI;

		// 0 .. 360 degrés ////////////////
		if (Theta < 0)
		{
			Theta = Theta + 360;
		}

		float Theta2 = Theta;
		if (Theta2 >= 180)
		{
			Theta2 = Theta2 - 360;
		}

		float Theta3 = Theta;
		if (Theta3 < 180)
		{
			Theta3 = Theta3 + 360;
		}
		///////////////////////////////////
		
		float AmpF = 0; float CmdY = 0; float CmdX = 0;

		AmpF = Amp;

		float PortY = 0; float PortX = 0;
		
		if (Theta > ZoneForw - DeltaForw / 2 && Theta < ZoneForw + DeltaForw / 2)
		{
			//Forward
			CmdY = AmpF;
		}
		else if (Theta > ZoneBack - DeltaBack / 2 && Theta < ZoneBack + DeltaBack / 2)
		{
			//Backward
			BACK = 1;
			Amin_D = Amin_BACK;
			Amax_D = Amax_BACK;
			CmdY = -AmpF;
		}
		else if (Theta > ZoneLeft - DeltaLeft / 2 && Theta < ZoneLeft + DeltaLeft / 2)
		{
			//Left
			CmdX = -AmpF;
		}
		else if (Theta < ZoneForw - DeltaForw / 2 && Theta > ZoneRight + DeltaRight / 2)
		{
			//For-Right
			if (DiagoActive == 1)
			{
				PortY = 1 - abs(Theta - (ZoneForw - DeltaForw / 2)) / abs((ZoneForw - DeltaForw / 2) - (ZoneRight + DeltaRight / 2));
				PortX = 1 - abs(Theta - (ZoneRight + DeltaRight / 2)) / abs((ZoneForw - DeltaForw / 2) - (ZoneRight + DeltaRight / 2));
				CmdY = AmpF * PortY;
				CmdX = AmpF * PortX;
			}
			else
			{
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta < ZoneLeft - DeltaLeft / 2 && Theta > ZoneForw + DeltaForw / 2)
		{
			//For-Left
			if (DiagoActive == 1)
			{
				PortX = 1 - abs(Theta - (ZoneLeft - DeltaLeft / 2)) / abs((ZoneLeft - DeltaLeft / 2) - (ZoneForw + DeltaForw / 2));
				PortY = 1 - abs(Theta - (ZoneForw + DeltaForw / 2)) / abs((ZoneLeft - DeltaLeft / 2) - (ZoneForw + DeltaForw / 2));
				CmdY = AmpF * PortY;
				CmdX = -AmpF * PortX;
			}
			else
			{
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta < ZoneBack - DeltaBack / 2 && Theta > ZoneLeft + DeltaLeft / 2)
		{
			//Back-Left
			BACK = 1;
			Amin_D = Amin_BACK;
			Amax_D = Amax_BACK; 
			if (DiagoActive == 1)
			{
				PortY = 1 - abs(Theta - (ZoneBack - DeltaBack / 2)) / abs((ZoneBack - DeltaBack / 2) - (ZoneLeft + DeltaLeft / 2));
				PortX = 1 - abs(Theta - (ZoneLeft + DeltaLeft / 2)) / abs((ZoneBack - DeltaBack / 2) - (ZoneLeft + DeltaLeft / 2));
				CmdY = -AmpF * PortY;
				CmdX = -AmpF * PortX;
			}
			else
			{
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta2 > Ri1 && Theta2 < (ZoneRight + DeltaRight / 2))
		{
			//Right
			CmdX = AmpF;
		}
		else if (Theta3 < Ri2 && Theta3 > ZoneBack + DeltaBack / 2)
		{
			//Back-Right
			BACK = 1;
			Amin_D = Amin_BACK;
			Amax_D = Amax_BACK;
			if (DiagoActive == 1)
			{
				PortX = 1 - abs(Theta3 - (Ri2)) / abs((Ri2)-(ZoneBack + DeltaBack / 2));
				PortY = 1 - abs(Theta3 - (ZoneBack + DeltaBack / 2)) / abs((Ri2)-(ZoneBack + DeltaBack / 2));
				CmdY = -AmpF * PortY;
				CmdX = AmpF * PortX;
			}
			else
			{
				CmdY = 0;
				CmdX = 0;
			}
		}

		if (fabsf(CmdX) < Amin)
		{
			CmdXout = 0;
		}
		else if (CmdX > Amax-0.15)
		{
			CmdXout = 1;
		}
		else if (CmdX < -Amax+0.15)
		{
			CmdXout = -1;
		}
		else if (CmdX >= 0)
		{
			CmdXout = ((CmdX - Amin) / (Amax - Amin)) + 0.15;
		}
		else if (CmdX < 0)
		{
			CmdXout = ((CmdX + Amin) / (Amax - Amin)) - 0.15;
		}

		if (BACK == 1 && abs(CmdY) < Amin_D)
		{
			CmdYout = 0;
		}
		else if (BACK == 0 && abs(CmdY) < Amin)
		{
			CmdYout = 0;
		}
		else if (CmdY > Amax-0.15)
		{
			CmdYout = 1;
		}
		else if (CmdY < -Amax_D+0.15)
		{
			CmdYout = -1;
		}
		else if (CmdY >= 0)
		{
			CmdYout = ((CmdY - Amin) / (Amax - Amin)) + 0.15;
		}
		else if (CmdY < 0)
		{
			CmdYout = ((CmdY + Amin_D) / (Amax - Amin_D)) - 0.15;
		}
		CmdYout = -CmdYout;

		CmdX_new = CmdXout * cos(Theta_offset* M_PI / 180) + CmdYout * sin(Theta_offset* M_PI / 180);
		CmdY_new = -CmdXout * sin(Theta_offset* M_PI / 180) + CmdYout * cos(Theta_offset* M_PI / 180);

		// Commande Z
		/*if (yaw >= NeutralUpDown - DeltaNeutralUpDown / 2 && yaw <= NeutralUpDown + DeltaNeutralUpDown / 2)
			UpDownComm = 0;
		else if (yaw > NeutralUpDown + DeltaNeutralUpDown / 2 && yaw < MaxUp)
			UpDownComm = (yaw - NeutralUpDown) / (MaxUp - NeutralUpDown);
		else if (yaw > MaxUp)
			UpDownComm = (MaxUp - NeutralUpDown) / (MaxUp - NeutralUpDown);
		else if (yaw < NeutralUpDown - DeltaNeutralUpDown / 2 && yaw > MaxDown)
			UpDownComm = -1 * (NeutralUpDown - yaw) / (NeutralUpDown - MaxDown);
		else if (yaw < MaxDown)
			UpDownComm = -1 * (NeutralUpDown - MaxDown) / (NeutralUpDown - MaxDown);*/

		RightLeftComm = CmdX_new;//CmdXout;//
		ForBackComm   = CmdY_new;//CmdYout;//
}
	
	
	
	/* #### END  ### */
float Robot_IMU_Control::C2_2(int var)
{
		float var1, var2;
		int mask = 32767; //mask 0x7FFF
		var1 = var & mask;
		var2 = (var >> 15)*(-32768);
		return (var1 + var2);
}

void Robot_IMU_Control::wrap_angle(float * p_angle)
{
		if (*p_angle > M_PI)
			*p_angle -= (2 * M_PI);
		if (*p_angle < M_PI)
			*p_angle += (2 * M_PI);
		if (*p_angle < 0)
			*p_angle += (2 * M_PI);
}

float Robot_IMU_Control::rad2deg(float angle_rad)
{
		return angle_rad *(180 / M_PI);
}

float Robot_IMU_Control::deg2rad(float angle_deg)
{
		return angle_deg *(M_PI / 180);
}

}
