#define _USE_MATH_DEFINES

#include <iostream>
#include <sstream>
#include <string>
//#include <random>
#include <stdio.h>
#include <stdlib.h>
//#include <intrin.h>
//#include "stdafx.h"
//#include "KinovaTypes.h"
//#include "omp.h"

//#include "Robot_IMU_Control.h"
//#include "Defined_Macro.h"
#include <libkindrv/kindrv.h>
#include <sys/select.h>

#define SPACEBAR 32
#define U 85
#define Q 81
#define W 87
#define E 69
#define D 68
#define F 70
#define Z 90
#define O 79
#define P 80
#define u 117
#define q 113
#define w 119
//#define e 101
#define d 100
#define f 102
#define z 122
#define o 111
#define p 112
//#define BAUDRATE  1998848
using namespace KinDrv;


	/* USB DEVICE */
	int result;  
	const char* usb_dev;
	std::string usb_name;
	
	/* END USB DEVICE DEFINITION */
	const int Packet_Size = 32;
	int PCKT [Packet_Size - 1];
	static int S_Of_F = 51;
	static int E_Of_F = 52;
	int Baudrate = 1998848;
	Robot_IMU_Control  CJACO;
	int JACO_PREVIOUS_MODE = 0;
	static bool ControlJACO = 0; //ON ou OFF
	static bool Acq_Started = 0; //ON ou OFF
	static bool Bouton_Hold = 0; //ON ou OFF
	//Byte Data_received = System::Convert::ToByte(0);
	//Byte Test;
	/////
	//Byte INIT_KY[256];
	/////
	//TIMING
	double START_TIME = 0;
	double END_TIME = 0;
	double GLOBAL_TIME;
	double INIT_TIME;
	int Sec, Min, Hr;
	int refresh_print = 0;
	int elapsed = 0;
	unsigned char data[Packet_Size+1];
	unsigned char* first;
	//KEYPAD
	int in_kyp; char ch_kyp;
	bool kyp_ready = 0; 
	int kyp_delay = 0;
	
	float keypressed[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	//PRINTING
	bool PRINT = 1;
	int flag=0;
	int c;
//#pragma intrinsic(radians)

	//[STAThread]
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
/* keyboard */
struct termios orig_termios;

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



/* Control */
jaco_joystick_axis_t Control(int b){
	/* init struct */ 
	jaco_joystick_axis_t axe;
	axe.trans_lr=0.0f;
	axe.trans_fb=0.0f;
	axe.trans_rot=0.0f;
	axe.wrist_fb=0.0f;
	axe.wrist_lr=0.0f;
	axe.wrist_rot=0.0f;
	return axe ;
}
 
 /* main */ 
int main(){
		/* keyboard playback configuration */
		
		
		/* end of keyboard playb/home/pi/libkindrv/examples/ex_main.cpp:579:1:ack configuration */

		
	//	std::random_device                  rand_dev;
	//	std::mt19937                        generator(rand_dev());
		
		//system("start /min /b XBOX\\JoyToKey.exe");
		
	//	printf("JoyToKey staint kbhit(void){


#pragma region PORT
		
		
		result=system("ls -l /dev/ttyUSB*");
		if (result!=0){
			std::cout <<"\n no USB devices were found\n" << std::endl;
			return 0;
		}
		else {
			printf("\nEnter /dev/ttyUSB* wished \n");
			std::cin >> usb_name;
			usb_dev=usb_name.c_str();
		}
		mySerial serial1(usb_dev,Baudrate);
	/*	if (!serial1.IsOpen()){
			printf("\n USB device is not opened, please retry \n");
			//serial1.Close();
			return 0;
		}
		*/
		//else {
			printf("\n USB Device is opened \n Press W to start acquisition \n");
			while ((ch_kyp != 'W') && (ch_kyp != 'w'))
			{
				std::cin >> ch_kyp;
			}
			printf("\n ACQUISITION DÉMARRÉE\n");
		//}
		elapsed = 10;
#pragma endregion


#pragma region loop
	while (1)
	{
		c=0;
		set_conio_terminal_mode();
		if (kbhit()){
			c=getch();
		}
		reset_terminal_mode();
		while(data[0] != 0x33) //&& data[Packet_Size] != 0x34)
		{
			serial1.Receive(data,1);
		}
		serial1.Receive(data,Packet_Size-1);
		
			if (data[Packet_Size-2] == 0x34)
			{
				refresh_print++;
				if (Acq_Started == 0)
				{
					Acq_Started = 1;
					printf("Acquisition Started Succesfully\n");
				}
				for (int kl = 0; kl < Packet_Size-1; kl++)
				{
					PCKT[kl] = (int) data[kl];	
				}
				for (int kl = 0; kl < Packet_Size-1; kl++)
				{
					data[kl] = 0;
				}
				CJACO.read_payload(PCKT);
				CJACO.sensor_fusion();
				CJACO.algo();

				if (refresh_print == elapsed)
				{
					if (PRINT)
					{
						printf("_______________________________________\n ");
						
						if (ControlJACO == 1)
							printf("API OUVERTE\n");
						else
							printf("API FERMÉE\n");
						if (Bouton_Hold == 1)
							printf("CONTROLE DÉMARRÉ\n");
						else
							printf("CONTROLE ARRETÉE\n");
						/*if (ControlJACO == 1)
							printf("PERIODE ENVOI CMD = %d ms \n " ,END_TIME * 1000);*/

						printf("FREQUENCE CAPTEUR = %d Hz \n", CJACO.FREQ);
						//Console::WriteLine("FREQUENCE BASE    = " + Convert::ToString(CJACO.FREQ_BASE) + " Hz");
						//Console::WriteLine("PITCH = " + Convert::ToString(CJACO.y) + " ____ ROLL = " + Convert::ToString(CJACO.x) + " ____ YAW = " + Convert::ToString(CJACO.yaw));
						printf("PITCH = %f ___ROLL____= %f \n "  ,CJACO.y , CJACO.x);
						printf("FORBACK = %f ____RIGHTLEFT_____=%f\n" , CJACO.ForBackComm,CJACO.RightLeftComm);
						printf("UpDownComm = %d " ,CJACO.UpDownComm);
						printf("ACC is : %f\n",atan2(-CJACO.acc[2], -CJACO.acc[0]));//CJACO.C2_2((PCKT[14] << 8) + PCKT[15]));
						
						
						//_________________________________________________________ TRY ___________________________________________________
						
						
					if (CJACO.y>=150 && flag==0){
					flag=1;
					KinDrv::init_usb();
					printf("Create a JacoArm \n");
					JacoArm *arm;
					try {
						arm = new JacoArm();
						printf("Successfully connected to JACO arm! \n");
					} catch( KinDrvException &e ) {
						printf("error %i: %s \n", e.error(), e.what());
						return 0;
					}

					printf("Gaining API control over the arm \n");
					arm->start_api_ctrl();
					
					//check if we need to initialize arm
					jaco_retract_mode_t mode = arm->get_status();
					printf("Arm is currently in state: %i \n", mode);
					if( mode == MODE_NOINIT ) {
					//push the "HOME/RETRACT" button until arm is initialized
						jaco_joystick_button_t buttons = {0};
						buttons[2] = 1;
						arm->push_joystick_button(buttons);

						while( mode == MODE_NOINIT ) {
							usleep(1000*10); // 10 ms
							mode = arm->get_status();
						}	

						arm->release_joystick();
					}
					printf("Arm is initialized now, state: %i \n", mode);

					// we want to start from home_position
					goto_home(arm);
					
					arm->set_control_cart();
					break;
				}	
						// __________________________________________________________ END ___________________________________________________
				
						//Console::WriteLine("YAW_FUSED_SENSOR = " + Convert::ToString(CJACO.yaw_fused_sensor) + " ____ YAW_FUSED_CPP = " + Convert::ToString(CJACO.yaw));
						//Console::WriteLine("GYROX = " + Convert::ToString(CJACO.gyrox));
						
						//Console::WriteLine("TEST RAD = " + Convert::ToString(radians(90)));
						//Console::WriteLine("CmdXout = " + Convert::ToString(CJACO.CmdXout) + " ____ CmdYout = " + Convert::ToString(CJACO.CmdXout));
						//Console::WriteLine("CmdX_new = " + Convert::ToString(CJACO.CmdX_new) + " ____ CmdY_new = " + Convert::ToString(CJACO.CmdX_new));
						//Console::WriteLine("Theta_offset = " + Convert::ToString(CJACO.Theta_offset));
					}
					refresh_print = 0;
				}
			}
			#pragma region CONTROL
			//initscr();
			//noecho();
			//nodelay(stdscr,TRUE);
			//scrollok(stdscr,TRUE);
			/*if (Bouton_Hold == 1 && ControlJACO == 1)
			{
				//Prendre en compte l'état du bouton avant d'arrêter un mouvement
				int result = (*MySendJoystickCommand)(CJACO.Control(1));
				if (result != 1)KinDrv::init_usb();
					printf("Create a JacoArm \n");
					JacoArm *arm;
					try {
						arm = new JacoArm();
						printf("Successfully connected to JACO arm! \n");
					} catch( KinDrvException &e ) {
						printf("error %i: %s \n", e.error(), e.what());
						return 0;
					}

					printf("Gaining API control over the arm \n");
					arm->start_api_ctrl();
					
					//check if we need to initialize arm
					jaco_retract_mode_t mode = arm->get_status();
					printf("Arm is currently in state: %i \n", mode);
					if( mode == MODE_NOINIT ) {
					//push the "HOME/RETRACT" button until arm is initialized
						jaco_joystick_button_t buttons = {0};
						buttons[2] = 1;
						arm->push_joystick_button(buttons);

						while( mode == MODE_NOINIT ) {
							usleep(1000*10); // 10 ms
							mode = arm->get_status();
						}	

						arm->release_joystick();
					}
					printf("Arm is initialized now, state: %i \n", mode);

					// we want to start from home_position
					goto_home(arm);
					
					arm->set_control_cart();
					
					printf("Command not transmitted to JACO, please retry...\n");
			}
			else if (Bouton_Hold == 0 && ControlJACO == 1)
			{
				int result = (*MySendJoystickCommand)(CJACO.Control(0));
				if (result != 1)
					printf("Command not transmitted to JACO, please retry...\n");
			}
			if (ControlJACO == 1)
			{-
				if (START_TIME != 0)
					END_TIME = omp_get_wtime() - START_TIME;
				START_TIME = omp_get_wtime();
			}*/
			#pragma endregion
			#pragma region KEYPRESS CALIBRATION/SETTING
			#pragma region REFRESH KYP
			if (kyp_delay != 0)
			{
				kyp_delay++;
				if (kyp_delay == 5)
					kyp_delay = 0;
			}
			#pragma endregion
			if (c==3){
				KinDrv::close_usb();
				break;
				}
			

			if (c==SPACEBAR && kyp_ready && (keypressed[0] == 0))//STOP/START API
			{
				keypressed[0] = 1;

				#pragma region STOP/START API
				kyp_delay = 1;
				if (ControlJACO == 1)
				{
				//	(*MySendJoystickCommand)(CJACO.Control(0));   -> need explanation here 
				//	(*MyMoveHome)();
				//	(*MyStopControlAPI)();ntrolJACO == 1)
				//	arm->stop_api_ctrl();
					
				//	int result = (*MyCloseAPI)();
					printf("JACO's control has been stopped...\n");
					ControlJACO = 0;
				//	serialPort1->ReadExisting();  -> need explanation
				}
				else
				{
					//FIXED BUG      
 
					ControlJACO = 1;
					
					KinDrv::init_usb();
					printf("Create a JacoArm \n");
					JacoArm *arm;
					try {
						arm = new JacoArm();
						printf("Successfully connected to JACO arm! \n");
					} catch( KinDrvException &e ) {
						printf("error %i: %s \n", e.error(), e.what());
						return 0;
					}

					printf("Gaining API control over the arm \n");
					arm->start_api_ctrl();
					
					//check if we need to initialize arm
					jaco_retract_mode_t mode = arm->get_status();
					printf("Arm is currently in state: %i \n", mode);
					if( mode == MODE_NOINIT ) {
					//push the "HOME/RETRACT" button until arm is initialized
						jaco_joystick_button_t buttons = {0};
						buttons[2] = 1;
						arm->push_joystick_button(buttons);

						while( mode == MODE_NOINIT ) {
							usleep(1000*10); // 10 ms
							mode = arm->get_status();
						}	

						arm->release_joystick();
					}
					printf("Arm is initialized now, state: %i \n", mode);

					// we want to start from home_position
					goto_home(arm);
					
					arm->set_control_cart();
					

				}

			}
			else
			{
				keypressed[0] = 0;
			}
			if (c==U || c==u)//LOCK/UNLOCK KEYPAD
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
			}
			else
			{
				keypressed[1] = 0;
			}
			if ((c==D || c==d) && kyp_ready)
			{
				if (keypressed[2] == 0)
				{
					#pragma region EN/DISABLE - SET 0
					kyp_delay = 1;
					if (Bouton_Hold == 1){
						Bouton_Hold = 0;
					}
					else
					{
						Bouton_Hold = 1;
						CJACO.xoff = CJACO.x0;
						CJACO.yoff = CJACO.y0;
						printf("NEUTRAL PITCH = %d\n",  CJACO.x0 );
						printf("NEUTRAL ROLL  = %d\n",  CJACO.x0 );
					}
					printf("BOUTON HOLD = %d\n" ,Bouton_Hold );
				
				}

				keypressed[2] = 1;
			}
			else
			{
				keypressed[2] = 0;
			}
			if ( (c==O || c==o) && kyp_ready)//CHANGE MODE 2 (BOUTON 1)
			{  
				if (keypressed[3] == 0)
				{
					#pragma region CHANGE MODE 2
					kyp_delay = 1;
					CJACO.PRESSED = 1;
					CJACO.B_INDEX = XY_BUTTON1;
					#pragma endregion
				}
				keypressed[3] = 1;
			}
			else
			{
				keypressed[3] = 0;
			}
			if ( (c==P || c==p) && kyp_ready)//CHANGE MODE 3 (BOUTON 3)
			{
				if (keypressed[4] == 0)
				{
					#pragma region CHANGE MODE 3
					kyp_delay = 1;
					CJACO.PRESSED = 1;
					CJACO.B_INDEX = XY_BUTTON3;
					#pragma endregion
				}
				keypressed[4] = 1;
			}
			else
			{
				keypressed[4] = 0;
			}
			if ((c==Z || c==z) && kyp_ready)//Calibrate NEUTRAL (BOUTON 5)
			{
				if (keypressed[5] == 0)
				{
					#pragma region CALIB NEUTRAL POSITION
					kyp_delay = 1;
					CJACO.xoff = CJACO.x0;
					CJACO.yoff = CJACO.y0;
					//CJACO.yawoff = CJACO.yaw;
					printf("NEUTRAL PITCH = %d\n" ,CJACO.x0 );
					printf("NEUTRAL ROLL  = %d \n" , CJACO.y0 );
					#pragma endregion
				}

				keypressed[5] = 1;
			}
			else
			{
				keypressed[5] = 0;
			}
			if ((c==F || c==f) && kyp_ready)//ENABLE/DISABLE PRINTING
			{
				if (keypressed[6] == 0)
				{
					#pragma region PRINTING
					kyp_delay = 1;
					if (PRINT == 1)
					{
						PRINT = 0;
						printf("AFFICHAGE INACTIF \n");
					}
					else
					{
						PRINT = 1;
						printf("AFFICHAGE ACTIVÉ \n");
					}
					#pragma endregion
				}

				keypressed[6] = 1;
			}
			else
			{
				keypressed[6] = 0;
			}
			
		}

		/* add by Quentin in order to close lisusb context */
		KinDrv::close_usb();
		/* end of adding */
		return 0;
	
}



