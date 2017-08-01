#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Defined_Macro.h"
#include "KinovaTypes.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"

namespace Control_IMU_JACO
{
	public class Control_JACO
	{
	public:

		IMU_Sensor HEAD_IMU;
		IMU_Sensor CHAIR_IMU;
		EMG_Sensor EMG_1;
		EMG_Sensor EMG_2;
		char HEAD_IMU_STATE = 0;
		char CHAIR_IMU_STATE = 0;
		char EMG_1_STATE = 0;
		char EMG_2_STATE = 0;
		char limit_missing;

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
		float y0;									//pitch brute	
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
		int ReferenceActive;						//indique l'acitivation du noeud de référence
		int Valid;									//indique la validité des marges et zones définies
		int DiagoActive;							//0 = diagonale désactivée, 1 = diagonale activée
		float Theta_offset;							//0 .. 360 degrés, mapping
		float ThRotReference;						//vitesse de rotation seuil maximale du fauteuil

		bool BACK = 0;
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
	};
}
