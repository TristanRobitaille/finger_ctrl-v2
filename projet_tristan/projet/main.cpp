
/***************************************************************************
 *  main.cpp - Projet de contrôle inertiel - Tristan
 *  Created: Tues July 25 2017
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <string>



#include <ctime>
#include <termios.h>

#include "Defined_Macro.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"
#include "Control_JACO.h"
#include "Dislin_Scope.h"
#include "exception.h"

#include <dislin.h>
#include <discpp.h>

#include <libkindrv/kindrv.h>
#include <sys/select.h>
#include "test_print.h"
#include "mySerial.h"

using namespace Control_IMU_JACO;
using namespace KinDrv;
using namespace std;

void update_console(void);
void update_plot (void);
int check_keypad(void);
int goto_home(JacoArm *arm);
void test_libkindrv (void);
void init_API (void);

struct termios orig_termios;
void reset_terminal_mode ();
void set_conio_terminal_mode ();
int kbhit ();
int getch ();
int result;
const char* usb_dev;
std::string usb_name;
int flag=0;
int c;

bool PORT_COM_OUVERT = 0;
const int Packet_Size = 32;
int PCKT [Packet_Size - 1];
unsigned char  data[33];
static int S_Of_F = 51;
static int E_Of_F = 52;
int JACO_PREVIOUS_MODE = 0;
static bool ControlJACO = 0; //ON ou OFF
static bool Acq_Started = 0; //ON ou OFF
static bool Bouton_Hold = 0; //ON ou OFF
char EVENT_SAFETY_KEY = NO_EVENT;
/////
unsigned char INIT_KY[256];
/////
int refresh = 0;
int elapsed = 10;

//KEYPAD
int in_kyp; char ch_kyp;
bool kyp_ready = 0;
int kyp_delay = 0;
const int nb_keys = 23;
float keypressed [nb_keys];

//PRINTING
bool PRINT = ON;
bool PRINT_CALIB = ON;

//PLOT
bool PLOT_DATA = OFF;

const int NB_SAMPLES_BY_PCKT = 9;
const int NB_SAMPLES_TO_PLOT = 11 * NB_SAMPLES_BY_PCKT;
const int NB_SCOPE_SAMPLES = 50 * NB_SAMPLES_TO_PLOT;

const int NB_TKE_TO_PLOT = 15;
const int NB_SCOPE_TKE = 50 * NB_TKE_TO_PLOT;

const int NB_IMU_TO_PLOT = 5;//2;//
const int NB_SCOPE_IMU = 50 * NB_IMU_TO_PLOT;// 10 * NB_IMU_TO_PLOT; //

Dislin_Scope PLOT_EMG;
float new_emg[NB_SAMPLES_TO_PLOT]; int i_nemg = 0;
float new2_emg[NB_SAMPLES_TO_PLOT]; int i_n2emg = 0;
Dislin_Scope PLOT_TKE;
float new_tke[NB_TKE_TO_PLOT]; int i_ntke = 0;
float th = 0;
Dislin_Scope PLOT_IMU;
float new_imu[4];

Control_JACO CTRL_IMU;

int mode_jaco_test;
int test_timer_jaco = 0;
unsigned int start_time;
unsigned int stop_time;
int BYTES_AVAILABLE;
int* pnbyrc = &BYTES_AVAILABLE;

bool TIMER_STARTED = 0;
bool JACO_STARTED = 0;
bool JACO_CONNECTED = 0;

JacoArm *arm;
jaco_joystick_t jojo_jaco = {0};

int main()
{
  printf(" ____     ___  ____  _____    _____  _____  _____  ____  ____  _____ _____ \n");
  printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
  printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
  printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
  printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
  printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

  if (JACO_CONNECTED)
  {
	init_API();
	/////  I  N  I  T       J  A  C  O       T  E  S  T        K  I  N  D  R  V   //////
	//test_libkindrv ();
	////////////////////////////////////////////////////////////////////////////////////
	//return 0;
  }

  CTRL_IMU.initialize();

  if (PLOT_DATA)
  {
		PLOT_EMG.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2);
		PLOT_TKE.initialize(TKE_SCOPE, 2, NB_SCOPE_TKE, NB_TKE_TO_PLOT, 2);
		PLOT_IMU.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU, NB_IMU_TO_PLOT, 2);
  }

  float FREQ;
  int i;

  ///////////////////////////////////////
  result=system("ls -l /dev/ttyUSB*");
  if (result != 0)
  {
	std::cout << "\n no USB devices were found\n" << std::endl;
	return 0;
  }
  else
  {
	//printf("\nEnter /dev/ttyUSB* wished \n");
	//std::cin >> usb_name;
	//usb_dev = usb_name.c_str();
	usb_dev = "/dev/ttyUSB0";
  }
	mySerial serial1(usb_dev, 1998848);//115200;//);

  while (1)
  {
	//read keypad
	c=0;
	set_conio_terminal_mode();
	if (kbhit())
	{
		c=getch();
	}
	reset_terminal_mode();

	//read data
	data[0] = 0;
	while(data[0] != 0x33) //&& data[Packet_Size] != 0x34)
	{
		serial1.Receive(data,1);
        //serial1.NumberByteRcv(*pnbyrc);
	}
	//serial1.NumberByteRcv(*pnbyrc);
	refresh ++;

	////TEST Timing///////////////////////////////////////////
	start_time = clock ();
	TIMER_STARTED = 1;
	//////////////////////////////////////////////////////////

	serial1.Receive(data,Packet_Size-1);//read all packet
	if (data[Packet_Size-2] == 0x34)
	{
		for (int kl = 0; kl < Packet_Size-1; kl++)
		{
			PCKT[kl] = (int) data[kl];
		}
		CTRL_IMU.process_payload(PCKT);


		//TEST TRISTAN
		////////////		Calibration 	//////////////
		if (CTRL_IMU.calDone <= 100) //First we run only this if() --> Nominal calibration
		{
			CTRL_IMU.y0Cal = CTRL_IMU.HEAD_IMU.rollraw;
			CTRL_IMU.y0Min = CTRL_IMU.HEAD_IMU.rollraw;
			CTRL_IMU.y0Max = CTRL_IMU.HEAD_IMU.rollraw;

			CTRL_IMU.x0Cal = CTRL_IMU.HEAD_IMU.yawraw;
			CTRL_IMU.x0Min = CTRL_IMU.HEAD_IMU.yawraw;
			CTRL_IMU.x0Max = CTRL_IMU.HEAD_IMU.yawraw;

			CTRL_IMU.calDone ++;
		}

		if (CTRL_IMU.calDone > 100 && CTRL_IMU.calDone <= 500){ //Then we run this second if() --> Min./Max. calibration
			CTRL_IMU.y0Temp = CTRL_IMU.HEAD_IMU.rollraw;
			CTRL_IMU.x0Temp = CTRL_IMU.HEAD_IMU.yawraw;

			if (CTRL_IMU.y0Temp < CTRL_IMU.y0Min){
			CTRL_IMU.y0Min = CTRL_IMU.y0Temp;
			CTRL_IMU.y0Min_LinCoef = (CTRL_IMU.y0Cal - CTRL_IMU.y0Min)/(-1*MAX_SPEED); //Compute linear coefficient to map Y angle to speed
			}

			if (CTRL_IMU.y0Temp > CTRL_IMU.y0Max){
			CTRL_IMU.y0Max = CTRL_IMU.y0Temp;
			CTRL_IMU.y0Max_LinCoef = (CTRL_IMU.y0Max - CTRL_IMU.y0Cal)/MAX_SPEED; //Compute linear coefficient to map Y angle to speed
			}

			if (CTRL_IMU.x0Temp < CTRL_IMU.x0Min){
			CTRL_IMU.x0Min = CTRL_IMU.x0Temp;
			CTRL_IMU.x0Min_LinCoef = (CTRL_IMU.x0Cal - CTRL_IMU.x0Min)/(-1*MAX_SPEED); //Compute linear coefficient to map X angle to speed
			}

			if (CTRL_IMU.x0Temp > CTRL_IMU.x0Max){
			CTRL_IMU.x0Max = CTRL_IMU.x0Temp;
			CTRL_IMU.x0Max_LinCoef = (CTRL_IMU.x0Max - CTRL_IMU.x0Cal)/MAX_SPEED; //Compute linear coefficient to map X angle to speed
			}
			
			if ((CTRL_IMU.calDone % 3) == 0){
			system("clear");
			printf("Nominal Calibration Done\n");
			printf("calDone Value: %d\n", CTRL_IMU.calDone);
			printf("Y-Axis Min./Max. Calibration Values: %f,\t%f\n", CTRL_IMU.y0Min, CTRL_IMU.y0Max);
			printf("x-Axis Min./Max. Calibration Values: %f,\t%f\n", CTRL_IMU.x0Min, CTRL_IMU.x0Max);
			printf("\n\n");
			printf("Y-Axis Positive/Negative Linear Coefficients: %f,\t%f\n", CTRL_IMU.y0Min_LinCoef, CTRL_IMU.y0Max_LinCoef);
			printf("X-Axis Positive/Negative Linear Coefficients: %f,\t%f\n", CTRL_IMU.x0Min_LinCoef, CTRL_IMU.x0Max_LinCoef);
			}
			
			CTRL_IMU.calDone ++;
		}

		if(CTRL_IMU.calDone > 500)
		{
			CTRL_IMU.finger_ctrl();
			update_plot();
			//printf("Normal while(1)");
			update_console ();
			if (TIMER_STARTED)
				stop_time = clock() - start_time;
				
				
			if (JACO_CONNECTED)
			{
				//////////////		 JACO Command 		//////////////
				jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
				jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
				arm->move_joystick_axis(jojo_jaco.axis);
				//jojo_jaco.axis.trans_lr = 0.0;
				//jojo_jaco.axis.trans_rot = 0.0;
				//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
				//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
			}
		}
	}

	
	
  }
  return 0;
}


void update_console(void)
{
#ifdef _WIN32
	if (START_TIME != 0)
		END_TIME = omp_get_wtime() - START_TIME;
	START_TIME = omp_get_wtime();
#endif
	if (refresh >= elapsed)
	{
		if (PRINT)
		{
#ifdef _WIN32
			system("cls");
#else
			system("clear");
#endif
			printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
			printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
			printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
			printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
			printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
			printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

			printf("___________________________________________________________________________\n");

			printf("LIMIT MISSING = %d\n", (int)CTRL_IMU.limit_missing);
			printf("GETCH c = %d\n", c);
			printf("Time Interval in millisecs : %d\n",stop_time);
			printf("Nb Bytes Received : %d\n\n", *pnbyrc);

			if (CTRL_IMU.calDone)
			printf("x0Cal = %f\n", CTRL_IMU.x0Cal);
			printf("y0Cal = %f\n", CTRL_IMU.y0Cal);
			printf("Y-Axis Positive/Negative Linear Coefficients: %f,\t%f\n", CTRL_IMU.y0Min_LinCoef, CTRL_IMU.y0Max_LinCoef);
			printf("X-Axis Positive/Negative Linear Coefficients: %f,\t%f\n", CTRL_IMU.x0Min_LinCoef, CTRL_IMU.x0Max_LinCoef);

			printf("___________________________________________________________________________\n");

			if (ControlJACO == 1)
				printf("API SET\n");
			else
				printf("API CLOSED\n");

			if (Bouton_Hold == 1)
				printf("CONTROL SET\n");
			else
				printf("CONTROL STOPPED\n");

			if (EVENT_SAFETY_KEY == ENTER_STANDBY)
				printf("SYSTEM IN STANDBY MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CALIBRATION)
				printf("SYSTEM IN CALIBRATION MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CONTROL)
				printf("SYSTEM IN CONTROL MODE\n");

			printf("variable test routine : %d\n", test_timer_jaco);

			printf("***************************************\n");
			if (CTRL_IMU.HEAD_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ HEADSET = %d Hz\n", CTRL_IMU.HEAD_IMU.FREQ);
				printf("RAW  : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchraw, (int)CTRL_IMU.HEAD_IMU.rollraw, (int)CTRL_IMU.HEAD_IMU.yawraw);
				printf("CTRL : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchctrl, (int)CTRL_IMU.HEAD_IMU.rollctrl, (int)CTRL_IMU.HEAD_IMU.yawctrl);
				printf("     : AMP   = %f __ THETA = %f | [deg]\n", (float)CTRL_IMU.Amp, (float)CTRL_IMU.Theta);
				if (CTRL_IMU.IMU_CALIBRATED)
					printf("HEADSET CALIBRATED\n");
				else
					printf("HEADSET NOT CALIBRATED\n");
			}
			else
				printf("HEADSET SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ CHAIR  = %d Hz\n", CTRL_IMU.CHAIR_IMU.FREQ);
				printf("HEADING JACO  : %d [deg]\n", (int)CTRL_IMU.CHAIR_IMU.yawraw);
				printf("CHAIR VELOCITY = %d deg/s   |  THRESHOLD = %d deg/s\n", (int)CTRL_IMU.CHAIR_IMU.MeanVelYaw, (int)CTRL_IMU.CHAIR_IMU.ThRotReference);
			}
			else
				printf("REFERENCE SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.EMG_1_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ EMG  = %d Hz\n", CTRL_IMU.EMG_1.FREQ * 9);
				printf("CURRENT TKE  = %f  |  THRESHOLD TKE = %f\n", CTRL_IMU.EMG_1.MEAN_TKE_CURRENT, CTRL_IMU.EMG_1.THRESHOLD);
				printf("UPPER THRES. = %f  |  LOWER THRES   = %f\n", CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD);
				printf("DETECTION EMG = %d |  CONTRACTION DURATION = %d\n ", CTRL_IMU.EMG_1.EMG_DETECTED, CTRL_IMU.EMG_1.time_contraction);
				if (CTRL_IMU.EMG_CALIBRATED)
					printf("EMG CALIBRATED\n");
				else
					printf("EMG NOT CALIBRATED\n");
			}
			else
				printf("EMG SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			//printf("PARAMS : Ctrl Z = %d | Ref active = %d | TH ROT CHAIR = %d deg/s (window = %d)\n", CTRL_IMU.ZActive, CTRL_IMU.ReferenceActive, (int)CTRL_IMU.CHAIR_IMU.ThRotReference, CTRL_IMU.CHAIR_IMU.nb_val_gyr);
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
				printf("HEADING OFFSET = %d\n", (int)CTRL_IMU.Theta_offset);
			//Console::WriteLine("DIRECTION = " + Convert::ToString(CTRL_IMU.DIRECTION)); //+ " ____ CmdYout = " + Convert::ToString(CJACO.CmdXout));
			printf("CMDX  = %f | CMDY = %f | CMDZ = %f \n", CTRL_IMU.Tristan_CmdX_new, CTRL_IMU.Tristan_CmdY_new, CTRL_IMU.Tristan_CmdZ_new);
			if (CTRL_IMU.ZActive)
				printf("Yaw Control Active\n");
			else
				printf("Yaw Control Not Active\n");

			printf("LIMIT MISSING = %d\n\n",(int)CTRL_IMU.limit_missing);


			printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
#ifdef _WIN32
			if (PRINT_CALIB)
			{
				printf("***************************************\n");
				Console::WriteLine("THETA     = " + Convert::ToString(CTRL_IMU.Theta) + "|  AMP = " + Convert::ToString(CTRL_IMU.Amp));
				Console::WriteLine("MODE ALGO = " + Convert::ToString(CTRL_IMU.MODE_JACO) + "|  MODE ROBOT = " + Convert::ToString(CTRL_IMU.MODE_JACO_RETURNED));
				printf("_______________________________________\n");
				Console::WriteLine("MIN FORW  = " + Convert::ToString(CTRL_IMU.AminForw) + "|  MIN BACK  = " + Convert::ToString(CTRL_IMU.AminBack));
				Console::WriteLine("MAX FORW  = " + Convert::ToString(CTRL_IMU.AmaxForw) + "|  MAX BACK  = " + Convert::ToString(CTRL_IMU.AmaxBack));
				Console::WriteLine("YAW FORW  = " + Convert::ToString(CTRL_IMU.YawCtrlForw) + "|  YAW BACK  = " + Convert::ToString(CTRL_IMU.YawCtrlBack));
				printf("_______________________________________\n");
				Console::WriteLine("MIN RIGHT = " + Convert::ToString(CTRL_IMU.AminRight) + "|  MIN LEFT  = " + Convert::ToString(CTRL_IMU.AminLeft));
				Console::WriteLine("MAX RIGHT = " + Convert::ToString(CTRL_IMU.AmaxRight) + "|  MAX LEFT  = " + Convert::ToString(CTRL_IMU.AmaxLeft));
				Console::WriteLine("YAW RIGHT  = " + Convert::ToString(CTRL_IMU.YawCtrlRight) + "|  YAW LEFT  = " + Convert::ToString(CTRL_IMU.YawCtrlLeft));
				printf("_______________________________________\n");
				Console::WriteLine("ROT MIN RIGHT = " + Convert::ToString(CTRL_IMU.RotminRight) + "|  ROT MIN LEFT  = " + Convert::ToString(CTRL_IMU.RotminLeft));
				Console::WriteLine("ROT MAX RIGHT = " + Convert::ToString(CTRL_IMU.RotmaxRight) + "|  ROT MAX LEFT  = " + Convert::ToString(CTRL_IMU.RotmaxLeft));
				Console::WriteLine("AMP ROT RIGHT = " + Convert::ToString(CTRL_IMU.AmpRotRight) + "|  AMP ROT LEFT   = " + Convert::ToString(CTRL_IMU.AmpRotLeft));
				printf("_______________________________________\n");
				Console::WriteLine("ZONE FORW  = " + Convert::ToString(CTRL_IMU.ZoneForw) + "|  ZONE BACK  = " + Convert::ToString(CTRL_IMU.ZoneBack));
				Console::WriteLine("ZONE RIGHT = " + Convert::ToString(CTRL_IMU.ZoneRight) + "|  ZONE LEFT  = " + Convert::ToString(CTRL_IMU.ZoneLeft));
			}
#endif
		}
		refresh = 0;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
void update_plot(void)
{
	if (PLOT_DATA)
	{
		if (PCKT[0] == HEADSET)
		{
			new_imu[0] = CTRL_IMU.HEAD_IMU.pitchctrl;
			new_imu[1] = CTRL_IMU.HEAD_IMU.rollctrl;
			new_imu[2] = CTRL_IMU.HEAD_IMU.yawctrl;
			PLOT_IMU.update(new_imu, 3, 1);
		}
		else if (PCKT[0] == REFERENCE)
		{
		}
		else if (PCKT[0] == EMG1)
		{
			if (CTRL_IMU.EMG_1.NEW_CALIB)
			{
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
			}
			for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
			{
				new_emg[i_nemg + pemg] = CTRL_IMU.EMG_1.DATA_EMG_FILT[pemg] * 5;
				if (new_emg[i_nemg + pemg] >= 100)
					new_emg[i_nemg + pemg] = 100;
				else if (new_emg[i_nemg + pemg] <= -100)
					new_emg[i_nemg + pemg] = -100;
			}
			i_nemg += NB_SAMPLES_BY_PCKT;

			new_tke[i_ntke] = CTRL_IMU.EMG_1.MEAN_TKE_CURRENT;
			i_ntke += 1;

			if (i_nemg >= PLOT_EMG.nb_data_to_plot)
			{
				PLOT_EMG.update(new_emg, PLOT_EMG.nb_data_to_plot, 1);
				i_nemg = 0;
			}
			if (i_ntke >= PLOT_TKE.nb_data_to_plot)
			{
				PLOT_TKE.update(new_tke, PLOT_TKE.nb_data_to_plot, 1);
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				i_ntke = 0;
			}
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////


/* keyboard */
void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}
void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}
int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

int check_keypad(void)
{
	int num_kyp = 0;
	if (kyp_delay != 0)
	{
		kyp_delay++;
		if (kyp_delay == 5)
			kyp_delay = 0;
	}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(SPACEBAR_KY) && kyp_ready && (keypressed[0] == 0))//STOP/START API
#else
	if (c == SPACEBAR_KY && kyp_ready && (keypressed[0] == 0))
#endif
	{
		keypressed[0] = 1;
		num_kyp = 1;
	#ifdef _WIN32
	#pragma region STOP/START API
		kyp_delay = 1;
		SET_JACO_ARM();
	#pragma endregion
	#endif
	}
	else {keypressed[0] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(U_KY)))//LOCK/UNLOCK KEYPAD
#else
	if (c == U_KY)
#endif
	{
		if (keypressed[1] == 0)
		{
			#pragma region LOCK/UNLOCK KEYPAD
			kyp_delay = 1;
			if (kyp_ready == 0)
			{
				kyp_ready = 1;
				printf("Keyboard Unlocked \n");
			}
			else
			{
				kyp_ready = 0;
				printf("Keyboard Locked \n");
			}
			#pragma endregion
		}
		keypressed[1] = 1;
		num_kyp = 2;
	}
	else {keypressed[1] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(D_KY) && kyp_ready)
#else
	if (c == D_KY && kyp_ready)
#endif
	{
		if (keypressed[2] == 0)
		{
			#pragma region EN/DISABLE - SET 0
			kyp_delay = 1;
			if (Bouton_Hold == 1)
				Bouton_Hold = 0;
			else
			{
				Bouton_Hold = 1;
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("NEUTRAL IMU CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
				}
				CTRL_IMU.InhibiteZctrl = 0;
			}
			//Console::WriteLine("BOUTON HOLD = " + Bouton_Hold + "\n");
			#pragma endregion
		}
		keypressed[2] = 1;
		num_kyp = 3;
	}
	else {keypressed[2] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(O_KY) && kyp_ready)//CHANGE MODE B (BOUTON 2)
#else
	if (c == O_KY && kyp_ready)
#endif

	{
		if (keypressed[3] == 0)
		{
			#pragma region CHANGE MODE 2
			kyp_delay = 1;
			if (ControlJACO)
			{
				CTRL_IMU.MODE_JACO += 1;
				if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
					CTRL_IMU.MODE_JACO = 0;

				CTRL_IMU.B_INDEX1 = XYZ_BUTTON2;
				CTRL_IMU.B_INDEX2 = 0;
				#ifdef _WIN32
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX1));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX1));
				#endif
			}
			#pragma endregion
		}
		keypressed[3] = 1;
		num_kyp = 4;
	}
	else {keypressed[3] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(P_KY) && kyp_ready)//CHANGE MODE A (BOUTON 1)
#else
	if (c == P_KY && kyp_ready)
#endif
	{
		if (keypressed[4] == 0)
		{
			#pragma region CHANGE MODE 3
			kyp_delay = 1;
			if (ControlJACO)
			{
				CTRL_IMU.MODE_JACO += 1;
				if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
					CTRL_IMU.MODE_JACO = 0;

				CTRL_IMU.B_INDEX1 = XYZ_BUTTON1;
				CTRL_IMU.B_INDEX2 = 0;
				#ifdef _WIN32
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX1));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX1));
				#endif
			}
			#pragma endregion
		}
		keypressed[4] = 1;
		num_kyp = 5;
	}
	else {keypressed[4] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(Z_KY) && kyp_ready)//Calibrate NEUTRAL (BOUTON 5)
#else
	if (c == Z_KY && kyp_ready)
#endif
	{
		if (keypressed[5] == 0)
		{
			#pragma region CALIB NEUTRAL POSITION
			kyp_delay = 1;
			CTRL_IMU.EMG_1.CALIB_EMG_INDEX = 0;
			if (CTRL_IMU.EMG_1.CALIB_EMG_INDEX != -1)
			{
				PRINT = 0;
				printf("NEUTRAL EMG CALIBRATION...\n");
			}
			#pragma endregion
		}

		keypressed[5] = 1;
		num_kyp = 6;
	}
	else {keypressed[5] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(F_KY) && kyp_ready)//ENABLE/DISABLE PRINTING
#else
	if (c == F_KY && kyp_ready)
#endif
	{
		if (keypressed[6] == 0)
		{
			kyp_delay = 1;
			if (PRINT == 1)
			{
				PRINT = 0;
				printf("AFFICHAGE INACTIF \n");
			}
			else
			{
				PRINT = 1;
				printf("AFFICHAGE ACTIVF \n");
			}
		}

		keypressed[6] = 1;
		num_kyp = 7;
	}
	else {keypressed[6] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(add_KY) && kyp_ready))//UP ARROW --> INCREASE EMG THRESHOLD
#else
	if (c == add_KY && kyp_ready)
#endif
	{
		if (keypressed[7] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.STD_TKE == 0)
					CTRL_IMU.EMG_1.STD_TKE = 1;
				else
					CTRL_IMU.EMG_1.K_TH++;
				printf("threshold increased...");
				CTRL_IMU.EMG_1.NEW_CALIB = 1;
			}
			keypressed[7] = 1;
			num_kyp = 8;
		}
	}
	else { keypressed[7] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(sub_KY) && kyp_ready))//DOWN ARROW -->	DECREASE EMG THRESHOLD
#else
	if (c == sub_KY && kyp_ready)
#endif
	{
		if (keypressed[8] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.K_TH == 1 && CTRL_IMU.EMG_1.STD_TKE != 0)
				{
					CTRL_IMU.EMG_1.STD_TKE--;
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
					printf("threshold decreased...");
				}
				else if (CTRL_IMU.EMG_1.K_TH != 1)
				{
					CTRL_IMU.EMG_1.K_TH--;
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
					printf("threshold decreased...");
				}
				else
					printf("not valid...");
			}
			keypressed[8] = 1;
			num_kyp = 9;
		}
	}
	else { keypressed[8] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(up_arrow) && kyp_ready))//MAX FORTH CALIBRATION
#else
	if (c == up_arrow && kyp_ready)
#endif
	{
		if (keypressed[9] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX FORWARD CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_FORTH;
			}
			keypressed[9] = 1;
			num_kyp = 10;
		}
	}
	else { keypressed[9] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(down_arrow) && kyp_ready))//MAX BACK CALIBRATION
#else
	if (c == down_arrow && kyp_ready)
#endif
	{
		if (keypressed[10] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX BACKWARD CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_BACK;
			}
			keypressed[10] = 1;
			num_kyp = 11;
		}
	}
	else { keypressed[10] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(right_arrow) && kyp_ready))//MAX RIGHT CALIBRATION
#else
	if (c == right_arrow && kyp_ready)
#endif
	{
		if (keypressed[11] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX RIGHT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_RIGHT;
			}
			keypressed[11] = 1;
			num_kyp = 12;
		}
	}
	else { keypressed[11] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(left_arrow) && kyp_ready))//MAX LEFT CALIBRATION
#else
	if (c == left_arrow && kyp_ready)
#endif
	{
		if (keypressed[12] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX LEFT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_LEFT;
			}
			keypressed[12] = 1;
			num_kyp = 13;
		}
	}
	else { keypressed[12] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad0) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad0 && kyp_ready)
#endif
	{
		if (keypressed[13] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("NEUTRAL IMU CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
			}
			keypressed[13] = 1;
			num_kyp = 12;
		}
	}
	else { keypressed[13] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad4) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad4 && kyp_ready)
#endif
	{
		if (keypressed[14] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("ROT MAX LEFT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_LEFT;
			}
			keypressed[14] = 1;
			num_kyp = 15;
		}
	}
	else { keypressed[14] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad6) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad6 && kyp_ready)
#endif
	{
		if (keypressed[15] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("ROT MAX RIGHT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_RIGHT;
			}
			keypressed[15] = 1;
			num_kyp = 16;
		}
	}
	else { keypressed[15] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(E_KY) && kyp_ready)//Z Control disabled/enabled
#else
	if (c == E_KY && kyp_ready)
#endif
	{
		if (keypressed[16] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.ZActive == 1)
			{
				CTRL_IMU.ZActive = 0;
				printf("Z Control disabled...\n");
			}
			else if (CTRL_IMU.ZActive == 0)
			{
				CTRL_IMU.ZActive = 1;
				printf("Z Control enabled...\n");
			}
		}

		keypressed[16] = 1;
		num_kyp = 17;
	}
	else { keypressed[16] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(V_KY) && kyp_ready)//Validate control heading offset
#else
	if (c == V_KY && kyp_ready)
#endif
	{
		if (keypressed[17] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("HEADING OFFSET CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = HEADING_OFFSET;
				SENSOR_HEADING_COMPUTATION;
				if (CTRL_IMU.CHAIR_IMU.SENSOR_ON)
					CTRL_IMU.ReferenceActive = 1;
			}
			keypressed[17] = 1;
			num_kyp = 18;
		}
	}
	else { keypressed[17] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad8) && kyp_ready))//LOWER_THRESHOLD INCREASE
#else
	if (c == numpad8 && kyp_ready)
#endif
	{
		if (keypressed[18] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.LOWER_THRESHOLD + 1 < CTRL_IMU.EMG_1.UPPER_THRESHOLD)
				{
					CTRL_IMU.EMG_1.LOWER_THRESHOLD += 1;
					printf("threshold increased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[18] = 1;
			num_kyp = 19;
		}
	}
	else { keypressed[18] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad2) && kyp_ready))//LOWER_THRESHOLD DECREASE
#else
	if (c == numpad2 && kyp_ready)
#endif
	{
		if (keypressed[19] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.LOWER_THRESHOLD - 1 >= 0)
				{
					CTRL_IMU.EMG_1.LOWER_THRESHOLD -= 1;
					printf("threshold decreased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[19] = 1;
			num_kyp = 20;
		}
	}
	else { keypressed[19] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad9) && kyp_ready))//UPPER_THRESHOLD INCREASE
#else
	if (c == numpad9 && kyp_ready)
#endif
	{
		if (keypressed[20] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				CTRL_IMU.EMG_1.UPPER_THRESHOLD += 1;
				printf("upper-threshold increased...");
				CTRL_IMU.EMG_1.NEW_CALIB = 1;
			}
			keypressed[20] = 1;
			num_kyp = 21;
		}
	}
	else { keypressed[20] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad3) && kyp_ready))//UPPER_THRESHOLD DECREASE
#else
	if (c == numpad3 && kyp_ready)
#endif
	{
		if (keypressed[21] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.UPPER_THRESHOLD - 1 > CTRL_IMU.EMG_1.LOWER_THRESHOLD)
				{
					CTRL_IMU.EMG_1.UPPER_THRESHOLD -= 1;
					printf("upper-threshold decreased...\n");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[21] = 1;
			num_kyp = 20;
		}
	}
	else { keypressed[21] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(W_KY) && kyp_ready))//Toggle Z control
#else
	if (c == W_KY && kyp_ready)
#endif
	{
		if (keypressed[22] == 0)
		{
			//PLOT_DATA = 0;
			if (CTRL_IMU.ZActive)
			{
				CTRL_IMU.ZActive = 0;
				printf("YAW CONTROL DISABLED...\n");
			}
			else
			{
				CTRL_IMU.ZActive = 1;
				printf("YAW CONTROL ENABLED...\n");
			}
			keypressed[22] = 1;
			num_kyp = 21;
		}
	}
	else { keypressed[22] = 0; }
	return num_kyp;
}

/* goto arm */

int goto_home(JacoArm *arm)
{
  // going to HOME position is possible from all positions. Only problem is,
  // if there is some kinfo of error
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      // is currently on the way to HOME. Need 2 button presses,
      // 1st moves towards RETRACT, 2nd brings it back to its way to HOME
      arm->push_joystick_button(2);
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_READY_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_READY_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
    if( mode == MODE_READY_TO_RETRACT ) {
      arm->release_joystick();
      arm->push_joystick_button(2);
    }
  }
  arm->release_joystick();

  return 1;
}

void test_libkindrv ()
{
	if (JACO_STARTED == 0)
	{
		KinDrv::init_usb();
		printf("Create a JacoArm \n");
		JacoArm *arm_test;
		try {
			arm_test = new JacoArm();
			printf("Successfully connected to JACO arm! \n");
		} catch( KinDrvException &e ) {
			printf("error %i: %s \n", e.error(), e.what());
			//return 0;
		}

		printf("Gaining API control over the arm \n");
		arm_test->start_api_ctrl();
		//jaco_retract_mode_t mode = arm->get_status();
		//goto_home(arm);
		arm_test->set_control_cart();

		printf("* translate forth \n");
	    MOVE_JACO_FORWARD;

		printf("* translate right \n");
		MOVE_JACO_RIGHT;

		printf("* translate back \n");
		MOVE_JACO_BACKWARD;

		printf("* translate left \n");
		MOVE_JACO_LEFT;

		printf("* translate UP \n");
		MOVE_JACO_UP;

		printf("* translate DOWN \n");
		MOVE_JACO_DOWN;

		JACO_STARTED = 1;
		//while (1);
	}
}

void init_API ()
{
	KinDrv::init_usb();
	printf("Create a JacoArm \n");
	try {
		arm = new JacoArm();
		printf("Successfully connected to JACO arm! \n");
	} catch( KinDrvException &e ) {
		printf("error %i: %s \n", e.error(), e.what());
		//return 0;
	}
	
	printf("Gaining API control over the arm \n");
	arm->start_api_ctrl();
	//jaco_retract_mode_t mode = arm->get_status();
	//goto_home(arm);
	arm->set_control_cart();
}

void init_joystick_cmd (jaco_joystick_t j)
{
	volatile unsigned int i;
	/// jaco_joystick_button_t button; /**< Simulated buttons. */
	/// \brief Simulation of joystick buttons (1 for pressed, 0 for released). Make sure to initialze with 0s!
	/// typedef unsigned short jaco_joystick_button_t[16];
	for (i = 0; i <16; i++)
		j.button [i] = 0;

	/// float axis[6]
	/// float trans_lr;    /**< (Translation Mode) Move stick +left -right. Left/Right translation of arm. */
    /// float trans_fb;    /**< (Translation Mode) Move stick +back -forth. Back/Forth translation to arm. */
    /// float trans_rot;   /**< (Translation Mode) Rotate stick +cw -ccw. Up/Down translation of arm. */
    /// float wrist_fb;    /**< (Wrist Mode) Move stick +forth -back. Up/Down inclination of wrist. */
    /// float wrist_lr;    /**< (Wrist Mode) Move stick +right -left. Forth/Back inclination of wrist. */
    /// float wrist_rot;   /**< (Wrist Mode) Rotate stick +cw -ccw. Ccw/cw rotation around wrist. */
	/// \brief Struct for joystick axis movement. Make sure to initialze with 0s!
	// jaco_joystick_axis_t   axis;   /**< Simulated axes. */
}
