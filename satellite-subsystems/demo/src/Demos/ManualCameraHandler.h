/*!
 * @file	ManualCameraHandler.h
 * @brief	helpful functions for manual image controlling and managment
 * @note	please advise the specified file name format when using this module
 */


#ifndef MANUAL_CAMERA_HANDLER_H_
#define MANUAL_CAMERA_HANDLER_H_

#include <satellite-subsystems/SCS_Gecko/gecko_driver.h>
#include <satellite-subsystems/SCS_Gecko/gecko_use_cases.h>

// TODO: all addresses and sizes should be in FRAM_Addresses.h
#define GECKO_STATUS_FLAG_ADDR 0x42
#define GECKO_STATUS_FLAG_SIZE 1

#define GECKO_ADC_GAIN_ADDR 0x43
#define GECKO_ADC_GAIN_SIZE 1

#define GECKO_PGA_GAIN_ADDR 0x44
#define GECKO_PGA_GAIN_SIZE 1

#define GECKO_EXPOSURE_ADDR 0x45
#define GECKO_EXPOSURE_SIZE 4

#define GECKO_FRAME_AMOUNT_ADDR 0x46
#define GECKO_FRAME_AMOUNT_SIZE 4

#define GECKO_FRAME_RATE_ADDR 0x47
#define GECKO_FRAME_RATE_SIZE 4

#define GECKO_FAST_ADDR 0x48
#define GECKO_FAST_SIZE 4

#define GECKO_MAX_ON_TIME_ADDR 0X49
#define GECKO_MAX_ON_TIME_SIZE 4
//-----------------------

#define IMAGE_SIZE (2048*1088)	///< image size in bytes

#define GECKO_ON  (0xFF)		///< flag- is the camera on
#define GECKO_OFF (0x00)		///< flag- is the camera off


// PLEASE NOT THAT IMAGE FILE FORMAT IS AS FOLLOWING: "imXXXX" where 'XXXX' is a 4byte unsigned integer id of image
#define MAX_IMAGE_FILENAME 15	///< maximum image file name is the OBC SD

typedef enum image_type_t
{
	fullsize,
	thumbnail2,
	thumbnail4,
	thumbnail8,
	thumbnail16,
	thumbnail32,
	thumbnail64,
	thumbnail128,
	thumbnail256,
	jpeg,
	rar
}image_type_t;


/*!
 * initializes GPIO 12 for spi communications with the camera
 */
void Initialized_GPIO();

/*!
 * De-initializes GPIO 12
 */
void De_Initialized_GPIO();


/*!
 * initializes the Gecko Camera including GPIO initialization, if no error occurred.
 * @return returns error according to 'gecko_use_cases.h' documentation
 */
int initGecko();

/*!
 * checks if the gecko camera is on or off
 * @return 	TRUE if the camera is on
 * 			FALSE if the camera is off
 */
Boolean isGeckoOn();

/*!
 * updates the FRAM parameter- maximum time the camera is on
 * @param[in] max_on_time maximum time (in millisec) in which the camera will be on.
 * @return	TRUE if successful update
 * 			FALSE if failed update
 * @warning this function does not check if value is valid or not. MAXIMUM VALUE MAY BE GIVEN!!
 */
Boolean updateGeckoMaxOnTime(portTickType max_on_time);

/*!
 * updates the FRAM parameters for taking image
 * @param[in] adcGain camera adc gain
 * @param[in] pgaGain camera pga gain
 * @param[in] exposure camera exposure
 * @param[in] frameAmount camera frame amount
 * @param[in] frameRate camera frame rate
 * @param[in] fast image reading parameter fast
 * @return	TRUE if successful update
 * 			FALSE if failed update in one or more parameters
 * @warning this function does not check if value is valid or not. MAXIMUM VALUE MAY BE GIVEN!!
 */
Boolean updateDefaultPictureParameters(uint8_t adcGain,uint8_t pgaGain,uint32_t exposure,
									uint32_t frameAmount,uint32_t frameRate, Boolean fast);

/*!
 * return the file descriptor of an image in the OBC SD.
 * @return pointer to file descriptor of image. returns error according to 'api_fat.h'. refer to fat api PDF for further help
 */
F_FILE* getImageFileFromID(unsigned int id,image_type_t im_type);

/*!
 * updates the FRAM parameters for taking image
 * @param[in] addr address of register
 * @param[out] data value of register
 * @param[in] endian data endian
 * @return returns error according to 'gecko_use_cases.h' documentation
 */
int get_gecko_register(char addr, unsigned int *data, GECKO_Endianess_t endian);

/*!
 * updates the FRAM parameters for taking image
 * @param[in] adcGain camera adc gain
 * @param[in] pgaGain camera pga gain
 * @param[in] exposure camera exposure
 * @return returns error according to 'gecko_use_cases.h' documentation
 * @warning this function does not check if value is valid or not. MAXIMUM VALUE MAY BE GIVEN!!
 */
int set_gecko_register(char addr, unsigned int data, GECKO_Endianess_t endian);

/*!
 * turns on Gecko Camera. gives power to the system.
 * @param[in] duration time span for which the camera will be on
 * @note if the camera is already on, the old delay will be the effective one.
 * you can abort the auto-shutdown using the designated function. see: 'abortGeckoAutuShutdown'
 */
void turn_gecko_on(portTickType duration);

/*!
 * turns off the Gecko camera. shuts down power to the camera
 * @note does not abort already commissioned auto shutdowns
 */
void turn_gecko_off();

/*!
 * aborts commissioned auto shutdown.
 * @warning NOW AUTO SHUTDOWN WILL NOT HAPPEN. USE CARFULLY!!
 */
void abortGeckoAutoShutdown();

/*!
 * Takes picture if camera is already on, otherwise turns the camera on and then takes picture
 * @param[in] id the unique ID to be given to the image
 * @param[in] max_gecko_on_time for how long will the camera be on, if not already on.
 * @note if the camera is already on, the old delay will be the effective one.
 * @return returns error according to 'GECKO_UC_TakeImage' in "gecko_use_cases.h"
 */
int take_picture_with_id(unsigned int id, portTickType max_gecko_on_time);

/*!
 * Takes picture if camera is already on, otherwise turns the camera on and then takes picture
 * @param[in] id the unique ID to be given to the image
 * @param[in] max_gecko_on_time for how long will the camera be on, if not already on.
 * @note if the camera is already on, the old delay will be the effective one.
 * @return returns error according to 'GECKO_UC_EraseBlock' in "gecko_use_cases.h"
 */
int delete_image_from_Gecko_SD(unsigned int id);

/*!
 * moves image from Gecko SD to OBC SD.
 * @param[in] id the unique ID to be given to the image
 * @return	returns error according to 'GECKO_UC_ReadImage' in "gecko_use_cases.h"
 * 			if no error in 'GECKO_UC_ReadImage' return -42 in file system error
 * 			0 if total success
 */
int move_image_to_OBC_SD(unsigned int id);

/*!
 * deletes image from Gecko SD to OBC SD.
 * @param[in] id the unique ID to be given to the image
 * @return	returns error according to 'api_fat.h'. refer to fat api PDF for further help
 */
int delete_image_from_OBC_SD(unsigned int id);

/*!
 * Compresses an image saved on OBC SD to a file according to it's compression algorithm
 * @param[in] id the unique ID to be given to the image
 * @param[in] im_type type of image to be compressed according to 'image_type_t' enum
 * @return	-1 error with opening image file -> 'f_open'
 * 			-2 error with reading image file -> 'f_read'
 * 			-3 error with writing to compressed image file -> 'f_write'
 * 			0 success
 */
int compress_image_on_OBC_SD(unsigned int id, image_type_t compression_alg, unsigned int *length);

#endif
