#include "IMU_Sensor.h"

namespace Control_IMU_JACO
{
	const double IMU_Sensor::num_lp_10Hz[3] = { 0.355, 0.355, 0.0 };
	const double IMU_Sensor::num_lp_25Hz[3] = { 0.755, 0.755, 0.0 };
	const double IMU_Sensor::den_lp_10Hz[3] = { 1.0, -0.2905, 0.0 };
	const double IMU_Sensor::den_lp_25Hz[3] = { 1.0, 0.5095, 0.0 };


	IMU_Sensor::IMU_Sensor()
	{}


	IMU_Sensor::~IMU_Sensor()
	{}


	void IMU_Sensor::initialize(char type)
	{
		pitchoffset = 0; rolloffset = 0; yawoffset = 0; yawrawmean = 500;
		amp_max_forw = 0; amp_max_back = 0; amp_max_right = 0; amp_max_left = 0;
		amp_min_forw = 0; amp_min_back = 0; amp_min_right = 0; amp_min_left = 0;
		yaw_ctrl_forw = 0; yaw_ctrl_back = 0; yaw_ctrl_right = 0; yaw_ctrl_left = 0;
		rot_max_right = 0; rot_max_left = 0; rot_min_right = 0; rot_min_left = 0;
		pitchrotright = 0; rollrotright = 0; pitchrotleft = 0; rollrotleft = 0;
		amp_rot_right = 0; amp_rot_right = 0;
		port_min_max = 0.1;

		CALIB_IMU_INDEX = -1; CALIB_TYPE = 0;
		if (type == HEADSET_IMU)
		{
			IMU_type = HEADSET_IMU;
			comp[0][0] = 0.000168679716274983;  comp[0][1] = -5.17710741425967e-06; comp[0][2] = -8.77884394210979e-07;
			comp[1][0] = -5.17710741425967e-06; comp[1][1] = 0.000159571222440490;  comp[1][2] = -3.27611632378113e-06;
			comp[2][0] = -8.77884394210986e-07; comp[2][1] = -3.27611632378113e-06; comp[2][2] = 0.000166456977421752;

			hardiron[0] = -725.100317519760; hardiron[1] = 491.703440889862; hardiron[2] = -53.3108814506022;

			init_rotmat[0][0] = 0; init_rotmat[0][1] = 0; init_rotmat[0][2] = -1;
			init_rotmat[1][0] = -1; init_rotmat[1][1] = 0; init_rotmat[1][2] = 0;
			init_rotmat[2][0] = 0; init_rotmat[2][1] = 1; init_rotmat[2][2] = 0;
		}
		else if (type == REFERENCE_IMU)
		{
			IMU_type = REFERENCE_IMU;
			comp[0][0] = 0.000161495931045042;  comp[0][1] = -4.43127013254319e-06; comp[0][2] = -2.96331601906327e-07;
			comp[1][0] = -4.43127013254319e-06; comp[1][1] = 0.000165988374412387;  comp[1][2] = -1.96475753260921e-06;
			comp[2][0] = -2.96331601906334e-07; comp[2][1] = -1.96475753260921e-06; comp[2][2] = 0.000164100497167486;

			hardiron[0] = -863.903533142847; hardiron[1] = 179.123382820601; hardiron[2] = 2556.53911843545;
		}
	}

	void IMU_Sensor::rotate_imu()
	{
		if (IMU_type == HEADSET)
		{
			float var0, var1, var2;
			var0 = acc[0]; var1 = acc[1]; var2 = acc[2];
			acc[0] = -1 * var2;
			acc[1] = -1 * var0;
			acc[2] = 1 * var1;

			var0 = gyr[0]; var1 = gyr[1]; var2 = gyr[2];
			gyr[0] = -1 * var2;
			gyr[1] = -1 * var0;
			gyr[2] = 1 * var1;

			var0 = mag[3]; var1 = mag[4]; var2 = mag[5];
			mag[3] = -1 * var2;
			mag[4] = -1 * var0;
			mag[5] = 1 * var1;
		}
	}

	void IMU_Sensor::read_new_data(int data[])
	{
		SENSOR_ON = 1;
		if (data[1] != 0)
		{
			FREQ = 1000 / data[1];											// MSP430 Sensor Acquisition Frequency
		}

		acc[0] = C2_2((data[16] << 8) + data[17]);
		acc[1] = C2_2((data[18] << 8) + data[19]);
		acc[2] = C2_2((data[20] << 8) + data[21]);

		gyr[0] = C2_2((data[10] << 8) + data[11]);
		gyr[1] = C2_2((data[12] << 8) + data[13]);
		gyr[2] = C2_2((data[14] << 8) + data[15]);
		if (IMU_type == REFERENCE_IMU)
		{
			moy_gyr[index_gyr] = abs(gyr[0]);
			index_gyr = index_gyr + 1;
		}

		mag[0] = C2_2((data[4] << 8) + data[5]);
		mag[1] = C2_2((data[6] << 8) + data[7]);
		mag[2] = C2_2((data[8] << 8) + data[9]);
	}

	void IMU_Sensor::calibrate_imu(void)
	{
		float magx_off, magy_off, magz_off;
		//gyr
		gyr[3] = gyr[0];
		gyr[4] = gyr[1];
		gyr[5] = gyr[2];

		//magnet
		magx_off = mag[0] - hardiron[0];
		magy_off = mag[1] - hardiron[1];
		magz_off = -mag[2] - hardiron[2];

		mag[3] = comp[0][0] * magx_off + comp[0][1] * magy_off + comp[0][2] * magz_off;
		mag[4] = comp[1][0] * magx_off + comp[1][1] * magy_off + comp[1][2] * magz_off;
		mag[5] = comp[2][0] * magx_off + comp[2][1] * magy_off + comp[2][2] * magz_off;
	}

	void IMU_Sensor::sensor_fusion(void)
	{
		//pitch
		pitch[0] = ALPHA_ACC*rad2deg(atan2(-acc[1], -acc[0])) + ALPHA_GYRO*(pitch[0] + (-gyr[2]) * GAIN_GYRO * (1 / FREQ_GYRO));
		x_pitch[0] = pitch[0];
		pitch[1] = num_lp_10Hz[0] * x_pitch[0] + num_lp_10Hz[1] * x_pitch[1] - den_lp_10Hz[1] * pitch[2];
		x_pitch[1] = x_pitch[0];
		pitch[2] = pitch[1];
		pitchraw = pitch[2];
		pitchctrl = pitchraw - pitchoffset;

		//roll
		roll[0] = ALPHA_ACC*rad2deg(atan2(-acc[2], -acc[0])) + ALPHA_GYRO*(roll[0] + gyr[1] * GAIN_GYRO * (1 / FREQ_GYRO));
		x_roll[0] = roll[0];
		roll[1] = num_lp_10Hz[0] * x_roll[0] + num_lp_10Hz[1] * x_roll[1] - den_lp_10Hz[1] * roll[2];
		x_roll[1] = x_roll[0];
		roll[2] = roll[1];
		rollraw = roll[2];
		rollctrl = rollraw - rolloffset;

		//yaw
		float cP, sP, cR, sR;
		cP = cos(deg2rad(-pitch[2]));
		sP = sin(deg2rad(-pitch[2]));
		cR = cos(deg2rad(roll[2]));
		sR = sin(deg2rad(roll[2]));

		xh_cal = cP*cR*mag[3] - sP*mag[4] + cP*sR*mag[5];
		yh_cal = sP*cR*mag[3] + cP*mag[4] + sP*sR*mag[5];
		zh_cal = -sR*mag[3] + 0 * mag[4] + cR*mag[5];

		yaw[0] = ALPHA_MAG*rad2deg(atan2(-zh_cal, -yh_cal)) + ALPHA_GYRO*(yaw[0] + gyr[0] * GAIN_GYRO * (1 / FREQ_GYRO));
		x_yaw[0] = yaw[0];
		yaw[1] = num_lp_10Hz[0] * x_yaw[0] + num_lp_10Hz[1] * x_yaw[1] - den_lp_10Hz[1] * yaw[2];
		x_yaw[1] = x_yaw[0];
		yaw[2] = yaw[1];
		yawraw = yaw[2];

		if (IMU_type == HEADSET_IMU)
		{
			//rotate yaw to zero
			float cPo, sPo, cRo, sRo, cYo, sYo;
			//cPo = cos(deg2rad(pitchoffset));  sPo = sin(deg2rad(pitchoffset));
			//cRo = cos(deg2rad(-rolloffset));  sRo = sin(deg2rad(-rolloffset)); 
			cPo = 1; sPo = 0; cRo = 1; sRo = 0;
			cYo = cos(deg2rad(-yawoffset));
			sYo = sin(deg2rad(-yawoffset));

			xh_ctrl = cPo*cRo			  *	xh_cal + sPo*cRo			  *	yh_cal - sRo	* zh_cal;
			yh_ctrl = -(sYo*cPo*sRo + cYo*sPo)*	xh_cal + (cYo*cPo - sYo*sPo*sRo)*	yh_cal - sYo*cRo* zh_cal;
			zh_ctrl = (cYo*cPo*sRo - sYo*sPo) *	xh_cal + (cYo*sPo*sRo + sYo*cPo)*	yh_cal + cYo*cRo* zh_cal;

			yaw_ctrl[0] = ALPHA_MAG_CTRL*rad2deg(atan2(-zh_ctrl, -yh_ctrl)) + ALPHA_GYRO_CTRL*(yaw_ctrl[0] + gyr[0] * GAIN_GYRO * (1 / FREQ_GYRO));
			x_yaw_ctrl[0] = yaw_ctrl[0];
			yaw_ctrl[1] = num_lp_10Hz[0] * x_yaw_ctrl[0] + num_lp_10Hz[1] * x_yaw_ctrl[1] - den_lp_10Hz[1] * yaw_ctrl[2];
			x_yaw_ctrl[1] = x_yaw_ctrl[0];
			yaw_ctrl[2] = yaw_ctrl[1];
			yawctrl = yaw_ctrl[2];
			//yawctrl = yawraw - yawoffset;

			if (CALIB_IMU_INDEX != -1 && CALIB_IMU_INDEX != -2)
			{
				float r_mean, p_mean;
				if (CALIB_TYPE == NEUTRAL || CALIB_TYPE == HEADING_OFFSET)
				{
					pitch_calib[CALIB_IMU_INDEX] = pitchraw;
					roll_calib[CALIB_IMU_INDEX] = rollraw;
					yaw_calib[CALIB_IMU_INDEX] = yawraw;
				}
				else
				{
					pitch_calib[CALIB_IMU_INDEX] = pitchctrl;
					roll_calib[CALIB_IMU_INDEX] = rollctrl;
					yaw_calib[CALIB_IMU_INDEX] = yawctrl;
				}
				CALIB_IMU_INDEX++;
				if (CALIB_IMU_INDEX >= NB_VALUES_CALIB)
				{
					CALIB_IMU_INDEX = -2;
					switch (CALIB_TYPE)
					{
					case NEUTRAL:
						pitchoffset = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						rolloffset = mean_tabf(roll_calib, NB_VALUES_CALIB);
						yawoffset = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					case MAX_FORTH:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_max_forw = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_forw = amp_max_forw*port_min_max;
						zone_forw = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_forw < 0)
							zone_forw += 360;
						yaw_ctrl_forw = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					case MAX_BACK:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_max_back = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_back = amp_max_back*port_min_max;//0.05;//
						zone_back = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_back < 0)
							zone_back += 360;
						yaw_ctrl_back = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					case MAX_RIGHT:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_max_right = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_right = amp_max_right*0.05;//port_min_max;
						zone_right = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_right < 0)
							zone_right += 360;
						yaw_ctrl_right = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					case MAX_LEFT:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_max_left = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_left = amp_max_left*port_min_max;
						zone_left = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_left < 0)
							zone_left += 360;
						yaw_ctrl_left = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					case MAX_ROT_RIGHT:
						rot_max_right = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						rot_min_right = rot_max_right*(port_min_max + 0.2);
						pitchrotright = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						rollrotright = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_rot_right = sqrt(pitchrotright*pitchrotright + rollrotright*rollrotright);
						//CALIB_TYPE = 0;
						break;
					case MAX_ROT_LEFT:
						rot_max_left = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						rot_min_left = rot_max_left*(port_min_max + 0.2);
						pitchrotleft = mean_tabf(pitch_calib, NB_VALUES_CALIB);
						rollrotleft = mean_tabf(roll_calib, NB_VALUES_CALIB);
						amp_rot_left = sqrt(pitchrotleft*pitchrotleft + rollrotleft*rollrotleft);
						//CALIB_TYPE = 0;
						break;
					case HEADING_OFFSET:
						yawrawmean = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						break;
					default:
						break;
					}
				}
			}
		}
		else if (IMU_type == REFERENCE_IMU)
		{
			if (index_gyr == nb_val_gyr)
				rotation_detected = detect_wheelchair_rotation();
			index_gyr = 0;
		}
	}

	char IMU_Sensor::detect_wheelchair_rotation()
	{
		//test rotation chaise
		MeanVelYaw = mean_tabf(moy_gyr, nb_val_gyr);
		char InhibiteZctrl = 0;
		if (MeanVelYaw >= ThRotReference)
			InhibiteZctrl = 1;
		return InhibiteZctrl;
	}

	float IMU_Sensor::mean_tabf(float * tab, int taille)
	{
		int g; float sum = 0.0;
		for (g = 0; g < taille; g++)
			sum = sum + tab[g];
		return (sum / taille);
	}

	float IMU_Sensor::C2_2(int var)
	{
		float var1, var2;
		int mask = 32767; //mask 0x7FFF
		var1 = var & mask;
		var2 = (var >> 15)*(-32768);
		return (var1 + var2);
	}

	float IMU_Sensor::rad2deg(float angle_rad)
	{
		return angle_rad *(180 / M_PI);
	}

	float IMU_Sensor::deg2rad(float angle_deg)
	{
		return angle_deg *(M_PI / 180);
	}
}