#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <satellite-subsystems/GomEPS.h>

#include <at91/peripherals/pio/pio.h>
#include <at91/boards/ISIS_OBC_G20/board.h>

#include <hcc/api_fat.h>

#include <hal/Drivers/SPI.h>
#include <hal/Storage/FRAM.h>

#include <stdint.h>

#include "ManualCameraHandler.h"

#define _SPI_GECKO_BUS_SPEED MHZ(5)

xTaskHandle _photographerHandle = NULL;

// todo: write compressed data directly into file
// todo: use the global buffer
unsigned char image[IMAGE_SIZE];
unsigned char buffer[IMAGE_SIZE];

F_FILE* getImageFileFromID(unsigned int id,image_type_t im_type)
{
	char filename[MAX_IMAGE_FILENAME + 1] = { 0 };
	switch (im_type)
	{
		case fullsize:
			snprintf(filename, MAX_IMAGE_FILENAME, "im%u", id);
			break;
		case thumbnail2:
			snprintf(filename, MAX_IMAGE_FILENAME, "tmb2_%u", id);
			break;
		case thumbnail4:
			snprintf(filename, MAX_IMAGE_FILENAME, "tmb4_%u", id);
			break;
		case thumbnail8:
			snprintf(filename, MAX_IMAGE_FILENAME, "tmb8_%u", id);
			break;
		case thumbnail16:
			snprintf(filename, MAX_IMAGE_FILENAME, "tmb16_%u", id);
			break;
		case thumbnail32:
			snprintf(filename, MAX_IMAGE_FILENAME, "tm32_%u", id);
			break;
		case thumbnail64:
			snprintf(filename, MAX_IMAGE_FILENAME, "tm64_%u", id);
			break;
		case thumbnail128:
			snprintf(filename, MAX_IMAGE_FILENAME, "tm128_%u", id);
			break;
		case thumbnail256:
			snprintf(filename, MAX_IMAGE_FILENAME, "tm256_%u", id);
			break;
		case jpeg:
			snprintf(filename, MAX_IMAGE_FILENAME, "jpg_%u", id);
			break;
		case rar:
			snprintf(filename, MAX_IMAGE_FILENAME, "rar_%u", id);
			break;
		default:
			snprintf(filename, MAX_IMAGE_FILENAME, "im%u", id);
			break;
	}

	F_FILE *fp_imfile = f_open(filename,"a+");
	return fp_imfile;
}

void updateGeckoStatusFlag(unsigned char flag)
{
	FRAM_write(&flag, GECKO_STATUS_FLAG_ADDR, GECKO_STATUS_FLAG_SIZE);
}

void Initialized_GPIO()
{
	Pin Pin12 = PIN_GPIO12;
	PIO_Configure(&Pin12, PIO_LISTSIZE(Pin12));
	vTaskDelay(10);
	PIO_Set(&Pin12);
	vTaskDelay(10);

}

void De_Initialized_GPIO()
{
	Pin Pin12 = PIN_GPIO12;
	PIO_Clear(&Pin12);
	vTaskDelay(10);
}

Boolean isGeckoOn()
{
	unsigned char gecko_status_flag = 0;

	if(0 != FRAM_read((unsigned char *)&gecko_status_flag, GECKO_STATUS_FLAG_ADDR, GECKO_STATUS_FLAG_SIZE))
	{
		return FALSE;
	}

return (gecko_status_flag == GECKO_ON) ? TRUE : FALSE;
}

int initGecko()
{
	int err = GECKO_Init( (SPIslaveParameters){ bus1_spi, mode0_spi, slave1_spi, 100, 1, _SPI_GECKO_BUS_SPEED, 0 } );
	if(0 == err)
	{
		Initialized_GPIO();
	}
	return err;
}

Boolean updateGeckoMaxOnTime(portTickType max_on_time)
{

	int err =FRAM_write((unsigned char*)&max_on_time,GECKO_MAX_ON_TIME_ADDR,GECKO_MAX_ON_TIME_SIZE);
	return (err==0)?TRUE:FALSE;
}

Boolean updateDefaultPictureParameters(uint8_t adcGain,uint8_t pgaGain,uint32_t exposure,uint32_t frameAmount,uint32_t frameRate, Boolean fast)
{
	char err = 0;
	err+= FRAM_write((unsigned char*)&adcGain,GECKO_ADC_GAIN_ADDR,GECKO_ADC_GAIN_SIZE);
	err+= FRAM_write((unsigned char*)&pgaGain,GECKO_PGA_GAIN_ADDR,GECKO_PGA_GAIN_SIZE);
	err+= FRAM_write((unsigned char*)&exposure,GECKO_EXPOSURE_ADDR,GECKO_EXPOSURE_SIZE);
	err+= FRAM_write((unsigned char*)&frameAmount,GECKO_FRAME_AMOUNT_ADDR,GECKO_FRAME_AMOUNT_SIZE);
	err+= FRAM_write((unsigned char*)&frameRate,GECKO_FRAME_AMOUNT_ADDR,GECKO_FRAME_AMOUNT_SIZE);
	err+= FRAM_read((unsigned char*)&fast,GECKO_FAST_ADDR,GECKO_FAST_SIZE);

	return (0 == err ? TRUE : FALSE);

}

int take_picture_with_id(unsigned int id, portTickType max_gecko_on_time)
{
	if (!isGeckoOn())
	{
		turn_gecko_on(max_gecko_on_time);
		vTaskDelay(10);
		initGecko();
		vTaskDelay(10);
	}

	uint8_t adcGain = 0;
	uint8_t pgaGain = 0;
	uint32_t exposure = 0;
	uint32_t frameAmount = 0;
	uint32_t frameRate = 0;
	Boolean fast = 0;
	FRAM_read((unsigned char*)&adcGain,GECKO_ADC_GAIN_ADDR,GECKO_ADC_GAIN_SIZE);
	FRAM_read((unsigned char*)&pgaGain,GECKO_PGA_GAIN_ADDR,GECKO_PGA_GAIN_SIZE);
	FRAM_read((unsigned char*)&exposure,GECKO_EXPOSURE_ADDR,GECKO_EXPOSURE_SIZE);
	FRAM_read((unsigned char*)&frameAmount,GECKO_FRAME_AMOUNT_ADDR,GECKO_FRAME_AMOUNT_SIZE);
	FRAM_read((unsigned char*)&frameRate,GECKO_FRAME_AMOUNT_ADDR,GECKO_FRAME_AMOUNT_SIZE);
	FRAM_read((unsigned char*)&fast,GECKO_FAST_ADDR,GECKO_FAST_SIZE);

	int err = GECKO_UC_TakeImage(adcGain,pgaGain,exposure,frameAmount, frameRate, (uint32_t) id);
	return err;
}

int move_image_to_OBC_SD(unsigned int id)
{
	GomEpsResetWDT(0); // 0 is the EPS index

	uint32_t buffer[IMAGE_SIZE];

	Boolean fast = 0;
	FRAM_read((unsigned char*)&fast,GECKO_FAST_ADDR,GECKO_FAST_SIZE);

	int err = GECKO_UC_ReadImage((uint32_t) id, buffer, fast);
	if(0 != err)
	{
		return err;
	}

	F_FILE *fp_imfile = getImageFileFromID(id,fullsize);
	if(NULL == fp_imfile)
	{
		return -42;
	}
	err = f_write(buffer,IMAGE_SIZE,1,fp_imfile);
	if(0 != err)
	{
		return -42;
	}
	f_close(fp_imfile);
	return 0;
}

int delete_image_from_OBC_SD(unsigned int id)
{
	char filename[MAX_IMAGE_FILENAME + 1] = { 0 };
	snprintf(filename, MAX_IMAGE_FILENAME, "im%u", id);

	int err = fm_delete(filename);
	return err;
}

int delete_image_from_Gecko_SD(unsigned int id)
{
	return GECKO_UC_EraseBlock((uint32_t)id);
}

int set_gecko_register(char addr, unsigned int data, GECKO_Endianess_t endian)
{
	return GECKO_SetReg((uint8_t)addr, (uint32_t)data, (GECKO_Endianess_t)endian);
}

int get_gecko_register(char addr, unsigned int *data, GECKO_Endianess_t endian)
{
	return GECKO_GetReg((uint8_t)addr, (uint32_t *)data, (GECKO_Endianess_t)endian);
}

void CameraReaperTask(void * camera_life_span)
{
	portTickType birth_date = 0;
	portTickType lifespan = *(portTickType*)camera_life_span;

	portTickType MAX_ON_TIME = 0;
	FRAM_read((unsigned char*)&MAX_ON_TIME,GECKO_MAX_ON_TIME_ADDR,GECKO_MAX_ON_TIME_SIZE);

	lifespan = (lifespan < MAX_ON_TIME)? lifespan : MAX_ON_TIME;

	while(1)
	{
		vTaskDelayUntil((portTickType * const)&birth_date, lifespan);
		turn_gecko_off();
		vTaskDelete(NULL); // commit suicide
	}
}

void turn_gecko_off()
{
	Pin Pin04 = PIN_GPIO04;
	Pin Pin05 = PIN_GPIO05;
	Pin Pin06 = PIN_GPIO06;
	Pin Pin07 = PIN_GPIO07;

	PIO_Clear(&Pin04);
	vTaskDelay(10);
	PIO_Clear(&Pin05);
	vTaskDelay(10);
	PIO_Clear(&Pin06);
	vTaskDelay(10);
	PIO_Clear(&Pin07);
	vTaskDelay(10);

	//todo: check if we need to de-initialize GPIO12
	updateGeckoStatusFlag(GECKO_OFF);
}

void abortGeckoAutoShutdown()
{
	if(NULL != _photographerHandle)
	{
		vTaskDelete(_photographerHandle);
	}
}

void turn_gecko_on(portTickType duration)
{
	//TODO: CHECK GECKO IS READY FOR ACTION

	Pin Pin04 = PIN_GPIO04;
	Pin Pin05 = PIN_GPIO05;
	Pin Pin06 = PIN_GPIO06;
	Pin Pin07 = PIN_GPIO07;
	PIO_Set(&Pin04);
	vTaskDelay(10);
	PIO_Set(&Pin05);
	vTaskDelay(10);
	PIO_Set(&Pin06);
	vTaskDelay(10);
	PIO_Set(&Pin07);
	vTaskDelay(10);

	updateGeckoStatusFlag(GECKO_ON);

	xTaskGenericCreate(CameraReaperTask, (const signed char*)"CameraReaperTask", 1000, (void*)&duration, configMAX_PRIORITIES-2, &_photographerHandle, NULL, NULL);
}

//compression_alg includes Thumbnails

Boolean Thumbnail(unsigned char image[IMAGE_SIZE], unsigned char *thumb, unsigned char thumb_factor)
{
	//TODO: copy code form gecko demo code
	return TRUE;
}

Boolean jpeg_compress(unsigned char image[IMAGE_SIZE], unsigned char *jpeg_im, unsigned int *length)
{
	//TODO: copy code from internet
	return TRUE;
}

Boolean lempelZivCompression(unsigned char image[IMAGE_SIZE], unsigned char *zip, unsigned int *length)
{
	return TRUE;
}

int compress_image_on_OBC_SD(unsigned int id, image_type_t compression_alg, unsigned int *length)
{
	if(compression_alg == fullsize)
	{
		return 0;
	}

	F_FILE *fp_image = getImageFileFromID(id,fullsize);
	F_FILE *fp_commpressed = getImageFileFromID(id,compression_alg);
	if(NULL == fp_image || NULL == fp_commpressed)
	{
		return -1;
	}
	f_close(fp_image);

	int err = f_read(image,IMAGE_SIZE,1,fp_image);
	if(1 != err)	// 1 is number of items read = 1 image = 1 item
	{
		return -2;
	}

	switch (compression_alg)
	{
		case fullsize:
			return 0;
		break;
		case thumbnail2:
			Thumbnail(image,buffer,2);
			*length = IMAGE_SIZE / (int)(1<<2);
			break;
		case thumbnail4:
			Thumbnail(image,buffer,3);
			*length = IMAGE_SIZE / (int)(1<<3);
			break;
		case thumbnail8:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<4);
			break;
		case thumbnail16:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<5);
			break;
		case thumbnail32:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<6);
			break;
		case thumbnail64:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<7);
			break;
		case thumbnail128:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<8);
			break;
		case thumbnail256:
			Thumbnail(image,buffer,4);
			*length = IMAGE_SIZE / (int)(1<<9);
			break;
		case jpeg:
			jpeg_compress(image,buffer,length);
			break;
		case rar:
			lempelZivCompression(image,buffer,length);
			break;
	}
	err = f_write(buffer,&length,1,fp_image);
	f_close(fp_commpressed);
	if(0 != err)
	{
		return -3;
	}
	return 0;
}

