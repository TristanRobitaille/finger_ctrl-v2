#define		XY				0
#define		XYZ				1
#define		WRIST			2
#define		FINGER			3
#define		XYZ_INDEX		1
#define		WRIST_INDEX		1
#define		FINGER_INDEX	6

#define		XY_BUTTON1		3
#define		XY_BUTTON2		1
#define		XY_BUTTON3		2
#define		XY_BUTTON_HOME	5

#define		XYZ_BUTTON1		0 //Change mode A (Translation/Wrist)
#define		XYZ_BUTTON2		1 //Change mode B (Finger)
#define		XYZ_BUTTON3		2 //HomeRetract 
#define		XYZ_BUTTON4		3 //DisableEnableJoystick
#define		XYZ_BUTTON5		4 //IncreaseSpeed
#define		XYZ_BUTTON6		5 //DecreaseSpeed

#define		POS				1
#define		NEG				0

#define		CMD_NULL		0
#define		CMD_FORWARD		1
#define		CMD_BACKWARD	2
#define		CMD_RIGHT		3
#define		CMD_LEFT		4
#define		CMD_FORRIGHT	5
#define		CMD_FORLEFT		6
#define		CMD_BACKRIGHT	7
#define		CMD_BACKLEFT	8

#define		M_PI				3.141592653589793238463  /* pi */
#define		SQRT_2				1.414213562  /* sqrt(2) */

#define		ALPHA_GYRO		0.92
#define		ALPHA_ACC		(1-ALPHA_GYRO)
#define		ALPHA_MAG		(1-ALPHA_GYRO)//0.08
#define		ALPHA_GYRO_CTRL	0.8	
#define		ALPHA_MAG_CTRL	(1-ALPHA_GYRO_CTRL)

#define		GAIN_GYRO		2000/32768
#define		FREQ_GYRO		62.5

#define		HEADSET			10
#define		REFERENCE		20
#define		EMG				30
#define		EMG1			31
#define		EMG2			32
#define		SAFETY_KEY_ID	40

#define		HEADSET_IMU		10
#define		REFERENCE_IMU	20

#define		EMG_SCOPE		1
#define		IMU_SCOPE		2
#define		TKE_SCOPE		3

#define		ON				1
#define		OFF				0

#define		NEUTRAL			0
#define		MAX_FORTH		1
#define		MAX_BACK		2
#define		MAX_RIGHT		3
#define		MAX_LEFT		4
#define		MAX_ROT_RIGHT	5
#define		MAX_ROT_LEFT	6
#define		HEADING_OFFSET	7

#define		FORWARD			1
#define		BACKWARD		2
#define		RIGHT			3
#define		LEFT			4
#define		FORRIGHT		5
#define		FORLEFT			6
#define		BACKRIGHT		7
#define		BACKLEFT		8

#define		SENSOR_HEADING_COMPUTATION		CTRL_IMU.HEAD_IMU.yawrawmean = 500
#define		SENSOR_HEADING_NOT_IN_PROGRESS	(z0mean != 500)

#define		START_CONTRACTION_CHRONO		time_contraction += 1
#define		LONG_CONTRACTION_DETECTED		time_contraction = -1
#define		SHORT_CONTRACTION_DETECTED		time_contraction = -2
#define		VERY_LONG_CONTRACTION_DETECTED	time_contraction = -3
#define		EMG_CONTRACTION_IS_LONG			(CTRL_IMU.EMG_1.time_contraction == -1)
#define		EMG_CONTRACTION_IS_SHORT		(CTRL_IMU.EMG_1.time_contraction == -2)
#define		EMG_CONTRACTION_IS_VERY_LONG	(CTRL_IMU.EMG_1.time_contraction == -3)
#define		RESET_CONTRACT_CHRONO			CTRL_IMU.EMG_1.time_contraction = 0	

#define		MODE_TRASNLATION		0
#define		MODE_ROTATION			1
#define		MODE_FINGER				2
#define		NB_MODES_JACO			3

#define		NO_EVENT				0
#define		ENTER_CALIBRATION		1
#define		ENTER_STANDBY			2
#define		ENTER_CONTROL			3

	////////////////////////////////////////
#define		SPACEBAR_KY		32
#define		U_KY			85
#define		Q_KY			81
#define		W_KY			87
#define		V_KY			86
#define		E_KY			69
#define		D_KY			68
#define		F_KY			70
#define		Z_KY			90
#define		O_KY			79
#define		P_KY			80
#define		u_KY			117
#define		q_KY			113
#define		w_KY			119
	//#define		v_KY			118
#define		e_KY			101
#define		d_KY			100
#define		f_KY			102
#define		z_KY			122
#define		o_KY			111
#define		p_KY			112
#define		up_arrow		38
#define		right_arrow		39
#define		down_arrow		40
#define		left_arrow		37

#define		numpad0			96
#define		numpad1			97
#define		numpad2			98
#define		numpad3			99
#define		numpad4			100
#define		numpad5			101
#define		numpad6			102
#define		numpad7			103
#define		numpad8			104
#define		numpad9			105

#define		add_KY			107
#define		sub_KY			109
///////////////////////////////////////