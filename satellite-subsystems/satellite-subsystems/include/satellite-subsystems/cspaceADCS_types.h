/*
 * eslADCS_types.h
 *
 *  Created on: 2 okt. 2014
 *      Author: malv
 */

#ifndef ESLADCS_TYPES_H_
#define ESLADCS_TYPES_H_

#include <hal/boolean.h>

//Telecommand parameter sizes
#define CSPACE_ADCS_SAVEIMAG_SIZE 	 		 	2
#define CSPACE_ADCS_GENPARAM_SIZE 	 		 	6
#define CSPACE_ADCS_CTRLPARAM_SIZE 	 		 	3
#define CSPACE_ADCS_ATTCTRLMODE_SIZE 	 		4
#define CSPACE_ADCS_RATESENCONF_SIZE 	 		7
#define CSPACE_ADCS_MAGOFFSCAL_SIZE 	 		12
#define CSPACE_ADCS_MAGSENSCONF_SIZE 	 		12
#define CSPACE_ADCS_ESTPARAM1_SIZE		 	 	16
#define CSPACE_ADCS_ESTPARAM2_SIZE		 	 	14
#define CSPACE_ADCS_CAMCONFPARAM_SIZE 	 		15
#define CSPACE_ADCS_STARTRK_CONFIG_SIZE 	 	26

//Telemetry reception sizes
#define CSPACE_ADCS_GENINFO_SIZE 	 	 		8
#define CSPACE_ADCS_EXCTIMES_SIZE 	 	 		8
#define CSPACE_ADCS_GENDATA_SIZE 	 		 	6
#define CSPACE_ADCS_MISCURR_SIZE 	 		 	4
#define CSPACE_ADCS_REDMAGMEAS_SIZE 	 		9
#define CSPACE_ADCS_GENCMDS_SIZE 	 		 	12
#define CSPACE_ADCS_CURRSTATE_SIZE				48
#define CSPACE_ADCS_MEASUREMENTS_SIZE			72
#define CSPACE_ADCS_RAWSENSORMEASUREMENT_SIZE	28
#define CSPACE_ADCS_POWTEMPMEAS_SIZE			34
#define CSPACE_ADCS_RAWGPSMEAS_SIZE				42
#define CSPACE_ADCS_ESTIMATION_METADATA_SIZE	42

/* Interface Enumerations*/

/// Run mode
typedef enum __attribute__ ((__packed__)) _cspace_adcs_runmode_t
{
	runmode_off = 0, ///< ADCS loop is inactive
	runmode_enabled = 1, ///< ADCS 1Hz loop is active
	runmode_triggered = 2, ///< ADCS will execute control loop only when triggered
	runmode_simulation = 3 ///< ADCS is in simulation mode
} cspace_adcs_runmode_t;

/// Power selection enumeration
typedef enum __attribute__ ((__packed__)) _cspace_adcs_pwsel_t
{
	selection_off = 0, ///< Permanently Off
	selection_on = 1, ///< Permanently On
	selection_curr_ctrl = 2, ///< Power state depends on current control mode
	selection_simulate = 3 ///< Simulate power control used for HIL simulations
} cspace_adcs_pwsel_t;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_capstat_t
{
	cap_startup = 0, ///< Start-up Capture pending
	cap_pending = 1, ///< Capture pending
	cap_succ_own = 2, ///< Successfully captured (own SRAM)
	cap_succ_shift = 3, ///< Successfully captured (other SRAM)
	cap_timeout = 4, ///< Camera timeout
	cap_sramerr = 5 ///< SRAM overcurrent
} cspace_adcs_capstat_t;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_detres_t
{
	det_startup = 0, ///< Start-up
	det_nodetec = 1, ///< No detection scheduled
	det_pending = 2, ///< Detection pending
	det_toomanyedges = 3, ///< Nadir error -too many detected edges
	det_toofewedges = 4, ///< Nadir error -not enough edges detected
	det_badfit = 5, ///< Nadir error -bad fit
	det_sunnotfound = 6, ///< Sun error -sun not found
	det_success = 7 ///< Successful detection
} cspace_adcs_detres_t;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_gps_solstat_t
{
	gps_solution_computed = 0, ///< Solution computed
	gps_solution_insufficientobs = 1, ///< Insufficient observations
	gps_solution_noconvergence = 2, ///< No convergence
	gps_solution_singularity = 3, ///< Singularity at parameters matrix
	gps_solution_covtrace_exceed = 4, ///< Covariance trace exceeds maximum
	gps_solution_notyetconverged = 5, ///< Not yet converged from cold start
	gps_solution_heightorvel_exceed = 6, ///< Height or velocity limits exceeded
	gps_solution_variance_exceed = 7, ///< Variance exceeds limits
	gps_solution_largeresidual = 8, ///< Large residuals make position unreliable
	gps_solution_calccomparison = 9, ///< Calculating comparison to user provided
	gps_solution_fixedposinvalid = 10, ///< The fixed position is invalid
	gps_solution_postypeunauthorized = 11 ///< Position type is unauthorized
} cspace_adcs_gps_solstat_t;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_exec_waypt_t
{
	execwayp_init = 0, ///< Busy with initialization
	execwayp_idle = 1, ///< Idle
	execwayp_senact_comm = 2, ///< Sensor/Actuator communications
	execwayp_adcs_update = 3, ///< ADCS estimation & control update
	execwayp_peripheral_pwrcmd = 4, ///< Peripheral power commands (over I2C)
	execwayp_cputemp_samp = 5, ///< CPU temperature sampling
	execwayp_image_dwl = 6, ///< Image download
	execwayp_image_compr = 7, ///< Image compression
	execwayp_sav_image_sdcard = 8, ///< Saving image to SD card
	execwayp_logging = 9, ///< Logging
	execwayp_log_filecompr = 10, ///< Log file compression
	execwayp_sav_log_sdcard = 11, ///< Saving log to SD card
	execwayp_writing_flash = 12 ///< Writing to flash memory
} cspace_adcs_exec_waypt_t;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_conmode_sel
{
	conmode_no_ctrl = 0, ///< No control
	conmode_det_ctrl = 1, ///< Detumbling control
	conmode_ythompson_spin = 2, ///< Y-Thompson spin
	conmode_ywheel_init_pitchacq = 3, ///< Y-Wheel momentum stabilized - Initial Pitch Acquisition
	conmode_ywheel_steadystate = 4, ///< Y-Wheel momentum stabilized - Steady State
	conmode_xyz_wheelctrl = 5, ///< XYZ - Wheel control
	conmode_rwheel_sun_trkctrl = 6, ///< Rwheel sun tracking control
	conmode_rwheel_target_trkctrl = 7, ///< Rwheel target tracking control
	conmode_vfast_spin_detctrl = 8, ///< 10Hz Detumbling control within Cube Control
	conmode_fast_spin_detctrl = 9, ///< Fast Detumbling control
	conmode_userdef_ctrlmode1 = 10, ///< User defined, or custom control mode 1
	conmode_userdef_ctrlmode2 = 11, ///< User defined, or custom control mode 2
	conmode_stop_rwheels = 12, ///< Stop all R-wheels
	conmode_usercod_ctrlmode = 13 ///< User coded control mode
} cspace_adcs_conmode_sel;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_estmode_sel
{
	estmode_noatt_est = 0, ///< No attitude estimation
	estmode_mems_rate = 1, ///< MEMS rate sensing
	estmode_mag_ratefilt = 2, ///< Magnetometer rate filter
	estmode_mag_ratefilt_pitch_est = 3, ///< Magnetometer rate filter with pitch estimation
	estmode_mag_fine_suntriad = 4, ///< Magnetometer and Fine-sun TRIAD algorithm
	estmode_full_state_ekf = 5, ///< Full-state EKF
	estmode_mems_gyro_ekf = 6 ///< MEMS gyro EKF
} cspace_adcs_estmode_sel;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_camsel_sel
{
	camsel_cam1 = 0, ///< cam1 camera
	camsel_cam2 = 1, ///< cam2 camera
	camsel_star = 2 ///< star camera
} cspace_adcs_camsel_sel;

typedef enum __attribute__ ((__packed__)) _cspace_adcs_imagsize
{
	imagsize_size0 = 0, ///< 1024 x 1024 pixels
	imagsize_size1 = 1, ///< 512 x 512 pixels
	imagsize_size2 = 2, ///< 256 x 256 pixels
	imagsize_size3 = 3, ///< 128 x 128 pixels
	imagsize_size4 = 4 ///< 64 x 64 pixels
} cspace_adcs_imagsize;

///////////////////////////////////////////////////////////

typedef union __attribute__ ((__packed__)) _cspace_adcs_refllhcoord_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short latitude; ///< Latitude angle ((formatted value) [deg] = RAWVAL * 0.01)
		short longitude; ///< Longitude angle ((formatted value) [deg] = RAWVAL * 0.01)
		short altitude; ///< Altitude angle ((formatted value) [deg] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_refllhcoord_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_ecirefvel_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_velocity; ///< x-velocity (measurement unit is [m/s] = RAWVAL * 0.25)
		short y_velocity; ///< y-velocity (measurement unit is [m/s] = RAWVAL * 0.25)
		short z_velocity; ///< z-velocity (measurement unit is [m/s] = RAWVAL * 0.25)
	} fields;
} cspace_adcs_ecirefvel_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_ecef_pos_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_ecefpos; ///< x-position (measurement unit is [m])
		short y_ecefpos; ///< y-position (measurement unit is [m])
		short z_ecefpos; ///< z-position (measurement unit is [m])
	} fields;
} cspace_adcs_ecef_pos_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_magfieldvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_magfield; ///< x-magnetic field ((formatted value)[uT] = RAWVAL * 0.01)
		short y_magfield; ///< y-magnetic field ((formatted value)[uT] = RAWVAL * 0.01)
		short z_magfield; ///< z-magnetic field ((formatted value)[uT] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_magfieldvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_sunvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_sun_cmp; ///< x-sun vector ((formatted value) = RAWVAL/10000.0)
		short y_sun_cmp; ///< y-sun vector ((formatted value) = RAWVAL/10000.0)
		short z_sun_cmp; ///< z-sun vector ((formatted value) = RAWVAL/10000.0)
	} fields;
} cspace_adcs_sunvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_nadirvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_nadir; ///< x-nadir vector ((formatted value) = RAWVAL/10000.0)
		short y_nadir; ///< y-nadir vector ((formatted value) = RAWVAL/10000.0)
		short z_nadir; ///< z-nadir vector ((formatted value) = RAWVAL/10000.0)
	} fields;
} cspace_adcs_nadirvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_pos_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_pos; ///< x-pos vector (measurement unit is [km] = RAWVAL * 0.25)
		short y_pos; ///< y-pos vector (measurement unit is [km] = RAWVAL * 0.25)
		short z_pos; ///< z-pos vector (measurement unit is [km] = RAWVAL * 0.25)
	} fields;
} cspace_adcs_pos_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_ratesen_temp_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_ratesen_temp; ///< x-rate sensor temp vector [C]
		short y_ratesen_temp; ///< y-rate sensor temp vector [C]
		short z_ratesen_temp; ///< z-rate sensor temp vector [C]
	} fields;
} cspace_adcs_ratesen_temp_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_attang_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short roll; ///< Roll angle (measurement unit is [deg] = RAWVAL * 0.01)
		short pitch; ///< Pitch angle (measurement unit is [deg] = RAWVAL * 0.01)
		short yaw; ///< Yaw angle (measurement unit is [deg] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_attang_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_angrate_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short x_angrate; ///< x angular rate (measurement unit is [deg/s] = RAWVAL * 0.01)
		short y_angrate; ///< y angular rate (measurement unit is [deg/s] = RAWVAL * 0.01)
		short z_angrate; ///< z angular rate (measurement unit is [deg/s] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_angrate_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_wspeed_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short speedX; ///< Wheel Speed x(measurement unit is [rpm])
		short speedY; ///< Wheel Speed y(measurement unit is [rpm])
		short speedZ; ///< Wheel Speed z(measurement unit is [rpm])
	} fields;
} cspace_adcs_wspeed_t;


typedef union __attribute__ ((__packed__)) _cspace_adcs_magnetorq_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short magX; ///< x-magnetorquer (RAWVAL = (formatted value) * 1000.0)
		short magY; ///< y-magnetorquer (RAWVAL = (formatted value) * 1000.0)
		short magZ; ///< z-magnetorquer (RAWVAL = (formatted value) * 1000.0)
	} fields;
} cspace_adcs_magnetorq_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_magtorqcmd_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short magXcmd; ///< x-magnetorquer (unit of measure is [10 ms units])
		short magYcmd; ///< y-magnetorquer (unit of measure is [10 ms units])
		short magZcmd; ///< z-magnetorquer (unit of measure is [10 ms units])
	} fields;
} cspace_adcs_magtorqcmd_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_starbody_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short startbX; ///< Start body X-vector ((formatted value) = RAWVAL/10000.0)
		short startbY; ///< Start body Y-vector ((formatted value) = RAWVAL/10000.0)
		short startbZ; ///< Start body Z-vector ((formatted value) = RAWVAL/10000.0)
	} fields;
} cspace_adcs_starbody_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_starorb_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short startoX; ///< Start orbit X-vector ((formatted value) = RAWVAL/10000.0)
		short startoY; ///< Start orbit Y-vector ((formatted value) = RAWVAL/10000.0)
		short startoZ; ///< Start orbit Z-vector ((formatted value) = RAWVAL/10000.0)
	} fields;
} cspace_adcs_starorb_t;

/*Telecommand data structures*/

typedef union __attribute__ ((__packed__)) _cspace_adcs_unixtm_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned int unix_time_sec; ///< time in s since 01/01/1970, 00:00 (measurement unit is [s])
		unsigned short unix_time_millsec; ///< current millisecond count (measurement unit is [ms])
	} fields;
} cspace_adcs_unixtm_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_powerdev_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_CTRLPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char signal_cubecontrol : 2, ///< Power CubeControl Signal
		motor_cubecontrol : 2, ///< Power CubeControl Motor
		pwr_cubesense : 2, ///< Power CubeSense
		pwr_cubestar : 2; ///< Power CubeStar
		unsigned char pwr_cubewheel1 : 2, ///< Power CubeWheel 1
		pwr_cubewheel2 : 2, ///< Power CubeWheel 2
		pwr_cubewheel3 : 2, ///< Power CubeWheel 3
		pwr_motor : 2; ///< Power Motor
		unsigned char pwr_gps; ///< Power GPS LNA
	} fields;
} cspace_adcs_powerdev_t;

/*Telemetry data structures*/

/** The eslADCS General Status*/
typedef union __attribute__ ((__packed__)) _cspace_adcs_geninfo_t
{
	/** Raw value array with general status data*/
	unsigned char raw[CSPACE_ADCS_GENINFO_SIZE];
	/** General Status values*/
	struct __attribute__ ((__packed__))
	{
		unsigned char node_type; ///< Node type identifier
		unsigned char version_interface; ///< Interface version. This field should have a value of 1.
		unsigned char version_major; ///< Firmware version (Major)
		unsigned char version_minor; ///< Firmware version (Minor)
		unsigned short uptime_secs; ///< Number of seconds since processor start-up
		unsigned short uptime_millisecs; ///< Number of milliseconds (after the integer second) since processor start-up
	} fields;
} cspace_adcs_geninfo_t;

/** The eslADCS Communication Status*/
typedef union __attribute__ ((__packed__)) _cspace_adcs_commstat_t
{
	/** Raw value array with communication status data*/
	unsigned char raw[CSPACE_ADCS_GENDATA_SIZE];
	/** Communication Status values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short tc_counter; ///< No. of telecommands received
		unsigned short tlm_reqcounter; ///< No. of telemetry requests received
		unsigned char tcbuffer_overrun : 1, ///< TC buffer was overrun while receiving a telecommand
		uart_protoc_err : 1, ///< UART protocol error occurred
		uart_incomp_message : 1, ///< UART start-of-message identifier was received
		i2cTLM_readerr : 1, ///< Number of data clocked out was more than tlm package
		i2cTC_bufferr : 1, ///< I2C Telecommand sent exceeds buffer size
		canTC_bufferr : 3; ///< CANTelecommand sent exceeds buffer size
		unsigned char reserved;
	} fields;
} cspace_adcs_commstat_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_currstate_t
{
	unsigned char raw[CSPACE_ADCS_GENDATA_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short attest_mode : 4, ///< Current attitude estimation mode
		ctrl_mode : 4, ///< Current attitude control mode
		run_mode : 2, ///< Current ADCS running mode
		cctrlsig_enabled : 1, ///< CubeControl Signal enable status
		cctrlmot_enabled : 1, ///< CubeControl Motor enable status
		csense_enabled : 1, ///< CubeSense enable status
		cwheel1_enabled : 1, ///< CubeWheel 1 enable status
		cwheel2_enabled : 1, ///< CubeWheel 2 enable status
		cwheel3_enabled : 1; ///< CubeWheel 3 enable status
		unsigned short cstar_enabled : 1, ///< CubeStar enable status
		gpsrx_enabled : 1, ///< GPS receiver enable status
		gpslna_enabled : 1, ///< GPS antenna LNA enable status
		motdriv_enabled : 1, ///< Motor driver enable status
		sunabovelh : 1, ///< Sun is above the local horizon enable status
		cs_commerr : 1, ///< Comm error occurred with the CubeSense
		cc_sigcomerr : 1, ///< Comm error occurred with the CubeControl Signal MCU
		cc_motcomerr : 1, ///< Comm error occurred with the CubeControl Motor MCU
		cwheel1_comerr : 1, ///< Comm error occurred with the CubeWheel 1
		cwheel2_comerr : 1, ///< Comm error occurred with the CubeWheel 2
		cwheel3_comerr : 1, ///< Comm error occurred with the CubeWheel 3
		cstar_comerr : 1, ///< Comm error occurred with the CubeStar
		mag_rangerr : 1, ///< Magnetometer measured magnetic field with size < 100 nT or > 100,000 nT
		cam1sen_overcurr : 1, ///< Cam1 sensor overcurrent detected
		cam1sen_busyerr : 1, ///< Cam1 sensor was not idle at the start of ADCS loop
		cam1sen_detecerr : 1; ///< Cam1 sensor was unable to compute angles (could be not in FOV)
		unsigned short sunsensor_rangerr : 1, ///< Detected sun angles were outside of +/- 90 deg
		cam2sen_overcurr : 1, ///< Cam2 sensor overcurrent detected
		cam2sen_busyerr : 1, ///< Cam2 sensor was not idle at the start of ADCS loop
		cam2sen_detetcerr : 1, ///< Cam2 sensor was unable to compute angles (could be not in FOV)
		nadirsensor_rangerr : 1, ///< Detected nadir angles were outside of +/- 60 deg
		ratesensor_rangerr : 1, ///< Measured XYZ-body rate is outside of the range +/- 20 deg/s
		wheelspeed_rangerr : 1, ///< Wheel XYZ speed measurement was outside of the range +/- 8500 rpm
		coarsunsensor_err : 1, ///< Unable to compute Coarse Sun vector (could be not in FOV)
		startrack_matcherr : 1, ///< Unable to obtain enough matched stars
		startrack_overcurr : 1, ///< Star tracker overcurrent detected
		orbitparam_invalid : 1, ///< Orbit Parameters are not in allowed bounds
		config_invalid : 1, ///< Magnetorquer configuration or CSS in invalid.
		ctrlmode_chgnotallow : 1, ///< Attempt was made to select control mode without appropriate estimator
		est_chgnotallow : 1, ///< Attempt was made to change to an estimation mode that would be inappropriate for the current ctrl mode
		mod_magfieldiff : 1, ///< Modelled and measured magnetic field differs in size by mode than 5000 nT
		noderec_err : 1; ///< Failed to Recover an ADCS Node by successive resets
	} fields;
} cspace_adcs_currstate_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_wheelcurr_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short wheel1Curr; ///< Wheel 1 Current ((formatted value)[mA] = RAWVAL * 0.01)
		short wheel2Curr; ///< Wheel 2 Current ((formatted value)[mA] = RAWVAL * 0.01)
		short wheel3Curr; ///< Wheel 3 Current ((formatted value)[mA] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_wheelcurr_t;

/** The eslADCS ACP execution state*/
typedef union __attribute__ ((__packed__)) _cspace_adcs_acp_info_t
{
	/** Raw value array with ACP execution state data*/
	unsigned char raw[sizeof(unsigned short) + 1];
	/** ACP execution state values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short time_start; ///< Time since the start of the current loop iteration [ms]
		cspace_adcs_exec_waypt_t curr_exec; ///< Indicates which part of the loop is currently executing
	} fields;
} cspace_adcs_acp_info_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawcam2_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short cam2_centroid_x; ///< Cam2 azimuth angle
		short cam2_centroid_y; ///< Cam2 elevation angle
		cspace_adcs_capstat_t capture_status; ///< Cam2 capture status
		cspace_adcs_detres_t detection_result; ///< Cam2 detection result
	} fields;
} cspace_adcs_rawcam2_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawcam1_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short cam1_centroid_x; ///< Cam1 azimuth angle
		short cam1_centroid_y; ///< Cam1 elevation angle
		cspace_adcs_capstat_t capture_status; ///< Cam1 capture status
		cspace_adcs_detres_t detection_result; ///< Cam1 detection result
	} fields;
} cspace_adcs_rawcam1_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_msctemp_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		short mcu_temp; ///< MCU Temperature. (formatted value)[C]
		short mag_temp; ///< Magnetometer Temperature. ((formatted value)[C] = RAWVAL / 10.0)
		short redmag_temp; ///< Redundant Magnetometer Temperature. ((formatted value)[C] = RAWVAL / 10.0)
	} fields;
} cspace_adcs_msctemp_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_actcmds_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENCMDS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_magnetorq_t magtorquer_cmds; ///< Magnetorquer commanded on-time
		cspace_adcs_wspeed_t wheel_speed_cmds; ///< Wheel speed commanded
	} fields;
} cspace_adcs_actcmds_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_measure_t
{
	/** Raw value array of data*/
	unsigned char raw[6 * CSPACE_ADCS_GENCMDS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_magfieldvec_t magfield; ///< Magnetic field vector
		cspace_adcs_sunvec_t course_sun; ///< Coarse sun sensor vector
		cspace_adcs_sunvec_t fine_sun; ///< Fine sun sensor vector
		cspace_adcs_nadirvec_t nadir_vector; ///< Nadir sensor vector
		cspace_adcs_angrate_t angular_rate; ///< Angular rate vector
		cspace_adcs_wspeed_t wheel_speed; ///< Wheel speed vector
		cspace_adcs_starbody_t star1b; ///< Star 1 body vector
		cspace_adcs_starorb_t star1o; ///< Star 1 orbit vector
		cspace_adcs_starbody_t star2b; ///< Star 2 body vector
		cspace_adcs_starorb_t star2o; ///< Star 2 orbit vector
		cspace_adcs_starbody_t star3b; ///< Star 3 body vector
		cspace_adcs_starorb_t star3o; ///< Star 3 orbit vector
	} fields;
} cspace_adcs_measure_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_igrf_magvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short igrf_magfield_x; ///< magnetic field x component ((formatted value)[uT] = RAWVAL * 0.01)
		short igrf_magfield_y; ///< magnetic field y component ((formatted value)[uT] = RAWVAL * 0.01)
		short igrf_magfield_z; ///< magnetic field z component ((formatted value)[uT] = RAWVAL * 0.01)
	} fields;
} cspace_adcs_igrf_magvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_sunmodvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short sunvecmodel_x; ///< sun modeled x component ((formatted value) = RAWVAL/10000.0)
		short sunvecmodel_y; ///< sun modeled y component ((formatted value) = RAWVAL/10000.0)
		short sunvecmodel_z; ///< sun modeled z component ((formatted value) = RAWVAL/10000.0)
	} fields;
} cspace_adcs_sunmodvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estgyrovec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short estgyrobias_x; ///< estimated rate sensor bias x component ((formatted value) [deg/s] = RAWVAL * 0.001)
		short estgyrobias_y; ///< estimated rate sensor bias y component ((formatted value) [deg/s] = RAWVAL * 0.001)
		short estgyrobias_z; ///< estimated rate sensor bias z component ((formatted value) [deg/s] = RAWVAL * 0.001)
	} fields;
} cspace_adcs_estgyrovec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estvec_innv_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short innovationvec_x; ///< estimation innovation x component ((formatted value) = RAWVAL * 0.0001)
		short innovationvec_y; ///< estimation innovation y component ((formatted value) = RAWVAL * 0.0001)
		short innovationvec_z; ///< estimation innovation z component ((formatted value) = RAWVAL * 0.0001)
	} fields;
} cspace_adcs_estvec_innv_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estqvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short estquaternion_q1; ///< quaternion error q1 component ((formatted value) = RAWVAL * 0.0001)
		short estquaternion_q2; ///< quaternion error q2 component ((formatted value) = RAWVAL * 0.0001)
		short estquaternion_q3; ///< quaternion error q3 component ((formatted value) = RAWVAL * 0.0001)
	} fields;
} cspace_adcs_estqvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_qerrvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short errquaternion_q1; ///< quaternion error q1 component ((formatted value) = RAWVAL * 0.0001)
		short errquaternion_q2; ///< quaternion error q2 component ((formatted value) = RAWVAL * 0.0001)
		short errquaternion_q3; ///< quaternion error q3 component ((formatted value) = RAWVAL * 0.0001)
	} fields;
} cspace_adcs_qerrvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_qcovvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short quatcovariancerms_q1; ///< quaternion covariance q1 component ((formatted value) = RAWVAL * 0.001)
		short quatcovariancerms_q2; ///< quaternion covariance q2 component ((formatted value) = RAWVAL * 0.001)
		short quatcovariancerms_q3; ///< quaternion covariance q3 component ((formatted value) = RAWVAL * 0.001)
	} fields;
} cspace_adcs_qcovvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_angrate_covvec_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short angratecov_x; ///< x angular rate covariance ((formatted value)[m] = RAWVAL * 0.001)
		short angratecov_y; ///< y angular rate covariance ((formatted value)[m] = RAWVAL * 0.001)
		short angratecov_z; ///< z angular rate covariance ((formatted value)[m] = RAWVAL * 0.001)
	} fields;
} cspace_adcs_angrate_covvec_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_stdev_pos_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short posX_stand_dev; ///< x position standard deviation ((formatted value)[m] = RAWVAL * 0.1)
		short posY_stand_dev; ///< y position standard deviation ((formatted value)[m] = RAWVAL * 0.1)
		short posZ_stand_dev; ///< z position standard deviation ((formatted value)[m] = RAWVAL * 0.1)
	} fields;
} cspace_adcs_stdev_pos_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_stdev_vel_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short velX_stand_dev; ///< x velocity standard deviation [m/s]
		short velY_stand_dev; ///< y velocity standard deviation [m/s]
		short velZ_stand_dev; ///< z velocity standard deviation [m/s]
	} fields;
} cspace_adcs_stdev_vel_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_statetlm_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_CURRSTATE_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_currstate_t curr_state; ///< Current state
		cspace_adcs_attang_t estim_angles; ///< Estimated angles
		cspace_adcs_estqvec_t est_quat; ///< Estimated quaternions
		cspace_adcs_angrate_t estim_angrate; ///< Estimated angular rate
		cspace_adcs_pos_t adcs_pos; ///< ECI referenced position
		cspace_adcs_ecirefvel_t adcs_vel; ///< ECI referenced velocity
		cspace_adcs_refllhcoord_t adcs_coord; ///< ADS coordinates
		cspace_adcs_ecef_pos_t ecef_pos; ///< ECEF position
	} fields;
} cspace_adcs_statetlm_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estmetadata_t
{
	unsigned char raw[CSPACE_ADCS_ESTIMATION_METADATA_SIZE];
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_igrf_magvec_t igrf_magfield; ///< IGRF Modelled magnetic field
		cspace_adcs_sunmodvec_t sunvecmodel; ///< Modelled sun vector
		cspace_adcs_estgyrovec_t estgyrobias; ///< Estimated gyro bias vector
		cspace_adcs_estvec_innv_t innovationvec; ///< Innovation vector
		cspace_adcs_qerrvec_t errquaternion; ///< Quaternion error vector
		cspace_adcs_qcovvec_t quatcovariancerms; ///< Quaternion covariance vector
		cspace_adcs_angrate_covvec_t angratecov; ///< Angular rate covariance vector
	} fields;
} cspace_adcs_estmetadata_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawcss1_6_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned char css_1; ///< Sampled A/D value (sun_angle)
		unsigned char css_2; ///< Sampled A/D value (sun_angle)
		unsigned char css_3; ///< Sampled A/D value (sun_angle)
		unsigned char css_4; ///< Sampled A/D value (sun_angle)
		unsigned char css_5; ///< Sampled A/D value (sun_angle)
		unsigned char css_6; ///< Sampled A/D value (sun_angle)
	} fields;
} cspace_adcs_rawcss1_6_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawcss7_10_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE - 2];
	struct __attribute__ ((__packed__))
	{
		unsigned char css_7; ///< Sampled A/D value (sun_angle)
		unsigned char css_8; ///< Sampled A/D value (sun_angle)
		unsigned char css_9; ///< Sampled A/D value (sun_angle)
		unsigned char css_10; ///< Sampled A/D value (sun_angle)
	} fields;
} cspace_adcs_rawcss7_10_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawmagmeter_t
{
	unsigned char raw[CSPACE_ADCS_GENDATA_SIZE];
	struct __attribute__ ((__packed__))
	{
		short magnetic_x; ///< Sampled A/D value
		short magnetic_y; ///< Sampled A/D value
		short magnetic_z; ///< Sampled A/D value
	} fields;
} cspace_adcs_rawmagmeter_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpssta_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_gps_solstat_t gps_status; ///< GPS solutions status
		unsigned char gps_satellites_tracked; ///< Number of tracked GPS satellites
		unsigned char gps_satellites_used; ///< Number of GPS satellites used in solution
		unsigned char gps_counter_xyzlog; ///< Counter for XYZ Lof from GPS
		unsigned char gps_counter_rangelog; ///< Counter for RANGE log from GPS
		unsigned char gps_logsetupmsg; ///< Response message for GPS log setup
	} fields;
} cspace_adcs_rawgpssta_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpstm_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		unsigned short gps_refweeks; ///< GPS reference week
		unsigned long gps_timemillisecs; ///< GPS time milliseconds [ms]
	} fields;
} cspace_adcs_rawgpstm_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpsx_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_x; ///< ECEF Position x. (Unit of measure is [m])
		short ecef_velocity_x; ///< ECEF Velocity x. (Unit of measure is [m/s])
	} fields;
} cspace_adcs_rawgpsx_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpsy_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_y; ///< ECEF Position y. (Unit of measure is [m])
		short ecef_velocity_y; ///< ECEF Velocity y. (Unit of measure is [m/s])
	} fields;
} cspace_adcs_rawgpsy_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpsz_t
{
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	struct __attribute__ ((__packed__))
	{
		long ecef_position_z; ///< ECEF Position z. (Unit of measure is [m])
		short ecef_velocity_z; ///< ECEF Velocity z. (Unit of measure is [m/s])
	} fields;
} cspace_adcs_rawgpsz_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawsenms_t
{
	unsigned char raw[CSPACE_ADCS_RAWSENSORMEASUREMENT_SIZE];
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_rawcam2_t cam2_raw; ///< Cam2 sensor raw measurements
		cspace_adcs_rawcam1_t cam1_raw; ///< Cam1 sensor raw measurements
		cspace_adcs_rawcss1_6_t css_raw1_6; ///< CSS raw measurements 1 to 6
		cspace_adcs_rawcss7_10_t css_raw7_10; ///< CSS raw measurements 7 to 10
		cspace_adcs_rawmagmeter_t magmeter_raw; ///< Magnetometer raw measurements
	} fields;
} cspace_adcs_rawsenms_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_rawgpsms_t
{
	unsigned char raw[CSPACE_ADCS_RAWGPSMEAS_SIZE];
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_rawgpssta_t gps_status; ///< GPS raw status
		cspace_adcs_rawgpstm_t gps_time; ///< GPS raw time
		cspace_adcs_rawgpsx_t gps_x; ///< GPS ECEF x axis
		cspace_adcs_rawgpsy_t gps_y; ///< GPS ECEF y axis
		cspace_adcs_rawgpsz_t gps_z; ///< GPS ECEF z axis
		cspace_adcs_stdev_pos_t pos_stdev; ///< Standard deviation position
		cspace_adcs_stdev_vel_t vel_stdev; ///< Standard deviation velocity
	} fields;
} cspace_adcs_rawgpsms_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_csencurrms_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short cs_3v3_current; ///< CubeSense 3V3 current ((formatted value)[mA] = RAWVAL * 0.1)
		short cs_cam2_sram_current; ///< CubeSense Nadir SRAM current ((formatted value)[mA] = RAWVAL * 0.1)
		short cs_cam1_sram_current; ///< CubeSense Sun SRAM current ((formatted value)[mA] = RAWVAL * 0.1)
	} fields;
} cspace_adcs_csencurrms_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_cctrlcurrms_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short cc_3v3_current; ///< CubeControl 3V3 current ((formatted value)[mA] = RAWVAL * 0.48828125)
		short cc_5v_current; ///< CubeControl 5V current ((formatted value)[mA] = RAWVAL * 0.48828125)
		short cc_Vbat_current; ///< CubeControl Vbat current ((formatted value)[mA] = RAWVAL * 0.48828125)
	} fields;
} cspace_adcs_cctrlcurrms_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_exctm_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_EXCTIMES_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short timeperf_adcsupdate; ///< Time to perform complete ADCS (Unit of measure is [ms])
		short timeperf_senactcom; ///< Time to perform Sensor/actuator comm (Unit of measure is [ms])
		short timeexc_sgp4prop; ///< Time to execute SGP4 propagator  (Unit of measure is [ms])
		short timeexc_igrfmodel; ///< Time to execute IGRF computation  (Unit of measure is [ms])
	} fields;
} cspace_adcs_exctm_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_misccurr_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_MISCURR_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		unsigned short cstar_curr; ///< CubeStar current ((formatted value)[mA] = RAWVAL * 0.01)
		unsigned short mag_curr; ///< Magnetorquer current ((formatted value)[mA] = RAWVAL * 0.1)
	} fields;
} cspace_adcs_misccurr_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_pwtempms_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_POWTEMPMEAS_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_csencurrms_t csense_curr; ///< CubeSense current measurements
		cspace_adcs_cctrlcurrms_t cctrl_curr; ///< CubeControl current measurements
		cspace_adcs_wheelcurr_t wheel_curr; ///< Wheels current measurements
		cspace_adcs_misccurr_t misc_curr; ///< Miscellaneous current measurements
		cspace_adcs_msctemp_t misc_temp; ///< Temperature telemetry
		cspace_adcs_ratesen_temp_t rate_sentemp; ///< Sensor rate temperature measurements
	} fields;
} cspace_adcs_pwtempms_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_attctrl_mod_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_ATTCTRLMODE_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		cspace_adcs_conmode_sel ctrl_mode; ///< Attitude control mode
		unsigned char override_flag; ///< Ignore current state and force control mode
		unsigned short timeout; ///< Control timeout duration. 0xFFF for infinite timeout. (Unit of measure is [s])
	} fields;
} cspace_adcs_attctrl_mod_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_camsencfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_CAMCONFPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short mounttrans_alpha_ang; ///< Mounting Transform Alpha Angle
		short mounttrans_beta_ang; ///< Mounting Transform Beta Angle
		short mounttrans_gamma_ang; ///< Mounting Transform Gamma Angle
		unsigned char detect_threshold; ///< Detection Threshold
		unsigned char auto_adjustmode; ///< Auto adjust mode
		unsigned short exposure_time; ///< Exposure time register value
		unsigned short boresight_x; ///< X pixel location of cam1 boresight x [pixels]. (raw parameter) = (formatted value) * 100.0
		unsigned short boresight_y; ///< Y pixel location of cam1 boresight Y [pixels]. (raw parameter) = (formatted value) * 100.0
		unsigned char cam_shift; ///< Shift processing chain
	} fields;
} cspace_adcs_camsencfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_magmountcfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_GENPARAM_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short mounttrans_alpha_ang; ///< Mounting Transform Alpha Angle. (raw parameter) = (formatted value [deg])*100.0
		short mounttrans_beta_ang; ///< Mounting Transform Beta Angle. (raw parameter) = (formatted value [deg])*100.0
		short mounttrans_gamma_ang; ///< Mounting Transform Gamma Angle. (raw parameter) = (formatted value [deg])*100.0
	} fields;
} cspace_adcs_magmountcfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_offscal_cfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_MAGOFFSCAL_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short mag_ch1off; ///< Magnetometer channel 1 offset. (raw parameter) = (formatted value [deg])*1000.0
		short mag_ch2off; ///< Magnetometer channel 2 offset. (raw parameter) = (formatted value [deg])*1000.0
		short mag_ch3off; ///< Magnetometer channel 3 offset. (raw parameter) = (formatted value [deg])*1000.0
		short mag_sensmatrix11; ///< Magnetometer Sensitivity Matrix 11. (raw parameter) = (formatted value [deg])*1000.0
		short mag_sensmatrix22; ///< Magnetometer Sensitivity Matrix 22. (raw parameter) = (formatted value [deg])*1000.0
		short mag_sensmatrix33; ///< Magnetometer Sensitivity Matrix 33. (raw parameter) = (formatted value [deg])*1000.0
	} fields;
} cspace_adcs_offscal_cfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_magsencfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_MAGSENSCONF_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short mag_sensmatrix_s12; ///< Magnetometer Sensitivity Matrix 12. (raw parameter) = (formatted value)*1000.0
		short mag_sensmatrix_s13; ///< Magnetometer Sensitivity Matrix 13. (raw parameter) = (formatted value)*1000.0
		short mag_sensmatrix_s21; ///< Magnetometer Sensitivity Matrix 21. (raw parameter) = (formatted value)*1000.0
		short mag_sensmatrix_s23; ///< Magnetometer Sensitivity Matrix 23. (raw parameter) = (formatted value)*1000.0
		short mag_sensmatrix_s31; ///< Magnetometer Sensitivity Matrix 31. (raw parameter) = (formatted value)*1000.0
		short mag_sensmatrix_s32; ///< Magnetometer Sensitivity Matrix 32. (raw parameter) = (formatted value)*1000.0
	} fields;
} cspace_adcs_magsencfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_ratesencfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_RATESENCONF_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short xrate_senoff; ///< x-rate sensor offset. (raw parameter) = (formatted value [deg/s])*1000.0
		short yrate_senoff; ///< y-rate sensor offset. (raw parameter) = (formatted value [deg/s])*1000.0
		short zrate_senoff; ///< z-rate sensor offset. (raw parameter) = (formatted value  [deg/s])*1000.0
		unsigned char rate_senmult; ///< Multiplier of rate sensor measurement
	} fields;
} cspace_adcs_ratesencfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_startrkcfg_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_STARTRK_CONFIG_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		short mounttrans_alpha_ang; ///< Mounting Transform Alpha Angle. (raw parameter) = (formatted value [deg])*100.0
		short mounttrans_beta_ang; ///< Mounting Transform Beta Angle. (raw parameter) = (formatted value [deg])*100.0
		short mounttrans_gamma_ang; ///< Mounting Transform Gamma Angle. (raw parameter) = (formatted value [deg])*100.0
		unsigned short exposure_time; ///< Exposure time register value
		unsigned char detect_threshold; ///< Detection Threshold
		unsigned char star_threshold; ///< Star Threshold
		unsigned char max_starmatched; ///< Maximum stars that the star tracker will match
		unsigned char max_starpixel; ///< Maximum pixels in a star
		unsigned char max_starnoise; ///< Maximum noise
		unsigned char min_starpixel; ///< Minimum pixels in a star
		unsigned char startrk_errmargin; ///< % Error margin of the star id. (raw parameter) = (formatted value [%])*100.0
		float startrk_centroidx; ///< Pixel centroid X
		float startrk_centroidy; ///< Pixel centroid Y
		unsigned short startrk_focal_leng; ///< Star Tracker Focal Length. (raw parameter) = (formatted value [mm])*10000.0
		unsigned char synch_delay; ///< Synchronization delay within star Tracker
	} fields;
} cspace_adcs_startrkcfg_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estparam1_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_ESTPARAM1_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		float mag_ratefilt_systnoise; ///< Magnetometer rate filter system noise.
		float ekf_sysnoise; ///< EKF system noise
		float css_measnoise; ///< CSS measurement noise
		float sunsen_measnoise; ///< Sun sensor measurement noise
	} fields;
} cspace_adcs_estparam1_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_estparam2_t
{
	/** Raw value array of data*/
	unsigned char raw[CSPACE_ADCS_ESTPARAM2_SIZE];
	/** Parameter values*/
	struct __attribute__ ((__packed__))
	{
		float nadirsen_measnoise; ///< Magnetometer rate filter system noise.
		float mag_measnoise; ///< EKF system noise
		float startrk_measnoise; ///< CSS measurement noise
		unsigned short use_sunsen; ///< Sun sensor measurement noise
	} fields;
} cspace_adcs_estparam2_t;

typedef union __attribute__ ((__packed__)) _cspace_adcs_savimag_t
{
	/** Raw value array of data*/
		unsigned char raw[CSPACE_ADCS_SAVEIMAG_SIZE];
		/** Parameter values*/
		struct __attribute__ ((__packed__))
		{
			cspace_adcs_camsel_sel camera_select; ///< Camera selection.
			cspace_adcs_imagsize image_size; ///< Image size selection.
		} fields;
} cspace_adcs_savimag_t;

#endif /* CSPACE_ADCS_TYPES_H_ */
