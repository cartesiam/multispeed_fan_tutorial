/*
*******************************************************************************
* @file   main.cpp
* @brief  Main program body
*******************************************************************************
* Main program for collecting data and testing NanoEdge AI solution
*
* Compiler Flags
* -DDATA_LOGGING : data logging mode for collecting data
* -DLIBRARY_MODE : classification mode with NanoEdge AI Library
*
* @note   if no compiler flag then data logging mode by default
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "LIS3DH.h"

#if !defined(DATA_LOGGING) && !defined(LIBRARY_MODE)
#define DATA_LOGGING
#endif

#ifdef LIBRARY_MODE
#include "NanoEdgeAI.h"
#include "knowledge.h"
#endif

/* Defines -------------------------------------------------------------------*/
#define BUFFER_SIZE 	512
#define NB_AXES 		3

/* Objects -------------------------------------------------------------------*/
UnbufferedSerial pc (USBTX, USBRX, 115200);
I2C lis3dh_i2c (D0, D1); // (I2C_SDA, I2C_SCL)

/* Sampling: 1600 Hz, Sensitivity: 4G */
LIS3DH lis3dh (lis3dh_i2c, LIS3DH_G_CHIP_ADDR, LIS3DH_DR_LP_1R6KHZ, LIS3DH_FS_4G);

/********************** Prototypes **********************/

void init (void);
#ifdef DATA_LOGGING
void data_logging_mode(void);
#endif
#ifdef LIBRARY_MODE
void library_mode(void);
#endif
void fill_acc_array (void);

/* Variables -----------------------------------------------------------------*/
float acc_x, acc_y, acc_z, last_acc_x, last_acc_y ,last_acc_z = 0;
float acc_buffer[BUFFER_SIZE * NB_AXES] = {0}, lis3dh_xyz[NB_AXES] = {0};
#ifdef LIBRARY_MODE
uint8_t similarity = 0;
uint16_t class_number;
#endif

/* BEGIN CODE-----------------------------------------------------------------*/
int main()
{
	/* Initialization */
	init();
	
#ifdef DATA_LOGGING
	/* Data logging mode */
	/* Compiler flag: -DDATA_LOGGING */
	data_logging_mode();
#endif

#ifdef LIBRARY_MODE
	/* NanoEdge AI Library classification mode */
	/* Compiler flag -DLIBRARY_MODE */
	library_mode();
#endif
}

/* Functions definition ------------------------------------------------------*/

void init ()
{
#ifdef LIBRARY_MODE
	NanoEdgeAI_knowledge_init(knowledge);
#endif
	if (lis3dh.read_id() != 0x33) {
		printf("ERROR: Accelerometer not found");
	}
}

#ifdef DATA_LOGGING
/* Code for data logging */
void data_logging_mode()
{
	while(1) {
		fill_acc_array();
	}
}
#endif

#ifdef LIBRARY_MODE
/* Code for classification using NanoEdge AI Library */
void library_mode()
{
	wait_us(1000000);
	while(1) {
		fill_acc_array();
		class_number = NanoEdgeAI_classifier(acc_buffer, output_class_buffer, 0);
		printf("Class detected: %s (Certainty: %d%%)\n", 
			id2class[class_number], 
			(uint16_t) (output_class_buffer[class_number - 1] * 100));
	}
}
#endif

/* Code for buiding accelerometer buffers using raw accelerometer data */
void fill_acc_array () // We get 256 samples of acc 3 axes
{
	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		if (lis3dh.data_ready()) { // New data is available
			lis3dh.read_data(&lis3dh_xyz[0]);
			acc_buffer[NB_AXES * i] = lis3dh_xyz[0];
			acc_buffer[(NB_AXES * i) + 1] = lis3dh_xyz[1];
			acc_buffer[(NB_AXES * i) + 2] = lis3dh_xyz[2];
		} else {
			i--; // New data not ready
		}
	}
#ifdef DATA_LOGGING
	/* Print accelerometer buffer, only for data logging */
	for (uint16_t isample = 0; isample < (NB_AXES * BUFFER_SIZE) - 1; isample++) {
		printf("%.4f ", acc_buffer[isample]);
	}
	printf("%.4f\n", acc_buffer[(NB_AXES * BUFFER_SIZE) - 1]);
#endif	
}
