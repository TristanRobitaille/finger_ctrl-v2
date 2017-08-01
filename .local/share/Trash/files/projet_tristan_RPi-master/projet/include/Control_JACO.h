#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Defined_Macro.h"
#include "KinovaTypes.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"

namespace Control_IMU_JACO
{
	class Control_JACO
	{
	public:

		IMU_Sensor HEAD_IMU;
		IMU_Sensor CHAIR_IMU;
		EMG_Sensor EMG_1;
		EMG_Sensor EMG_2;
		int HEAD_IMU_STATE;
		int CHAIR_IMU_STATE;
		int EMG_1_STATE;
		int EMG_2_STATE;
		int limit_missing;

		//DIRECTION - COMMANDES
		float CmdX_new; float CmdY_new;
		float CmdXout;	float CmdYout;
		float CmdZ; float CmdZ_new;
		float RightLeftComm; float ForBackComm; float UpDownComm;
		float RightLeftComm_ctrl; float ForBackComm_ctrl; float UpDownComm_ctrl;
		char B_INDEX, B_INDEX1, B_INDEX2;

		//Data 2D
		float x;									//roll + offset
		float y;									//pitch + offset
		float x0;									//roll brute
		float y0;									//pitch brut
		float xoff;									//offset - position neutre roll
		float yoff;									//offset - position neutre pitch
		float Amp;
		float Theta;
		float Orientation;
		int DIRECTION;

		// Z
		float z0, z0mean;
		float z_ctrl; int cpt_neutral;
		float zoff;
		float z0_ref;
		int InhibiteZctrl;

		//CALIBRATION**************************************************************************************/
		int ZActive;
		int ReferenceActive;						//indique l'acitivation du noeud de r�f�rence
		int Valid;									//indique la validit� des marges et zones d�finies
		int DiagoActive;							//0 = diagonale d�sactiv�e, 1 = diagonale activ�e
		float Theta_offset;							//0 .. 360 degr�s, mapping
		float ThRotReference;						//vitesse de rotation seuil maximale du fauteuil

		bool BACK;
		float Amin_BACK;
		float Amax_BACK;
		float Amin_D;
		float Amax_D;
		float Amax;								//inclinaison maximale
		float Amin;									//seuil d'inclinaison

		float ZoneForw, DeltaForw;	//Angle Forward	, Marge Forwad
		float ZoneRight, DeltaRight;	//Angle Right	, Marge Right
		float ZoneLeft, DeltaLeft;	//Angle Left	, Marge Left
		float ZoneBack, DeltaBack;	//Angle Backward, Marge Backward

		float AmaxForw, AmaxBack;
		float AminForw, AminBack;
		float AmaxRight, AmaxLeft;
		float AminRight, AminLeft;

		float YawCtrlForw, YawCtrlBack;
		float YawCtrlRight, YawCtrlLeft;

		float RotmaxRight, RotmaxLeft;
		float RotminRight, RotminLeft;
		float PitchRotRight, RollRotRight;
		float PitchRotLeft, RollRotLeft;
		float AmpRotRight, AmpRotLeft;

		int MODE_JACO, MODE_JACO_RETURNED;

		float DeltaRot;

		bool IMU_CALIBRATED, EMG_CALIBRATED;
		char imu_calibration[7];

		////////////////////TRISTAN//////////////////////
		int calDone;
		int minMaxCal;
		int Jaco_Init;
		float y0Temp;
		float y0Cal;
		float y0Min;
		float y0Max;
		float y0Min_LinCoef;
		float y0Max_LinCoef;
		float x0Temp;
		float x0Cal;
		float x0Min;
		float x0Max;
		float x0Min_LinCoef;
		float x0Max_LinCoef;
		float Tristan_CmdX_new;
		float Tristan_CmdY_new;
		float Tristan_CmdZ_new;
		/**************************************************************************************************/

		Control_JACO();
		~Control_JACO();
		void initialize(void);
		void set_default_calibration(void);
		void process_payload(int *PCKT);
		int update_calibration(void);
		void algo(void);
		float fmaxf(float x, float y);
		int ReadJACOMode(void);
		JoystickCommand CommandJACO(int b);
		JoystickCommand PushButton(char buttonindex);
		JoystickCommand ReleaseButton(char buttonindex);

		//TRISTAN
		void finger_ctrl (void);
	};
}
