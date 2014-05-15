/*

	Test code for using ufo daughter board as slave. Master is initialized but unused.
	Collect sensor data and sid's are mockups.

*/
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include <string.h>
#include <util/delay.h>	


/*! Defining an example slave address. */
// this is the slave address of the device itself, not a slave it talks to
#define SLAVE_ADDRESS    0x55

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED   2000000
#define BAUDRATE	 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/*! Defining number of bytes in local arrays. */
#define SENSOR_NVAL     1
#define DEBUG_NVAL	    2  // debug >= data
#define DATA_BYTES      4  // 4 = float, 2 = int16, 1 = byte

#define FIRMWARE_VERSION 0
#define HARDWARE_VERSION 0

/*! MPL defines */
#define MPL_ADDRESS 0x60
float P, T;

/*! Metallux Defines */
#define METALLUX_ADDRESS 0x78
float Ap = 0.05; 
float At = 0.10; 
float Bp = 0.95;
float Bt = 0.90;
int Pmax = 5;
int Pmin = 0;
int Tmax = 125;
int Tmin = -40;

/* Global variables */
TWI_Master_t twieMaster;    /*!< TWI master module. */
TWI_Slave_t twicSlave;      /*!< TWI slave module. */

float data_values[DEBUG_NVAL] = {-99.0, -99.0};
float null_value = -99.0;
uint8_t sensor_sid[SENSOR_NVAL] = {0x12};
uint8_t debug_sid[DEBUG_NVAL] = {0x12, 0x13};

float * return_datum;

uint8_t data_ready = 0;

uint8_t command = 0;
uint8_t command_argument = 0;
uint8_t iSID = 0;


uint8_t MPL3115A2_readSensors(float *P_kPa, float *T_C){
	// uint8_t twi_status = (uint8_t)TWI_MasterState(&twieMaster);
	// return twi_status;

	uint8_t command[2] = {0x26, 0x38}; /* Set to Barometer with an Oversample = 128 */
	//uint8_t success = TWI_MasterWrite(&twieMaster, MPL_ADDRESS, command, 2);

	uint8_t success = TWI_MasterRead(&twieMaster, MPL_ADDRESS, 8);

	(*P_kPa) = twieMaster.readData[0]; 	
	(*T_C) = twieMaster.readData[1];
	return success;

	if (success != 1) return twieMaster.result+20;

	command[0] = 0x13; /* Enable Data Flags in PT_DATA_CFG */
	command[1] = 0x07;
	success = TWI_MasterWrite(&twieMaster, MPL_ADDRESS, command, 2);
	if (success != 1) return twieMaster.result+30;

	command[0] = 0x26; // CTRL_REG1
	command[1] = 0x39; /* Set Active */
	success = TWI_MasterWrite(&twieMaster, MPL_ADDRESS, command, 2);
	if (success != 1) return twieMaster.result+40;

	uint8_t mpl_ready = 0;
	while (!mpl_ready){
		success = TWI_MasterRead(&twieMaster, MPL_ADDRESS, 1); 
		uint8_t mpl_status = twieMaster.readData[0];
		mpl_ready = ((mpl_status & 0x08)>>4) == 1;
	}
	
	success = TWI_MasterRead(&twieMaster, MPL_ADDRESS, 6); 
	uint8_t mpl_status = twieMaster.readData[0];
	uint8_t OUT_P_MSB = twieMaster.readData[1];
	uint8_t OUT_P_CSB = twieMaster.readData[2];
	uint8_t OUT_P_LSB = twieMaster.readData[3];
	uint8_t OUT_T_MSB = twieMaster.readData[4];
	uint8_t OUT_T_LSB = twieMaster.readData[5];

	uint32_t P_Pa = (OUT_P_MSB<<16) | (OUT_P_CSB<<8) | (OUT_P_LSB); // value in Pa * 64
	// (*P_kPa) = (float)P_Pa / 64. / 1000.;

	uint16_t T_raw = ((OUT_T_MSB<<8) | OUT_T_LSB);
	// (*T_C) = (float)T_raw / 256.;

    return twieMaster.bytesRead;

}

uint8_t Metallux_readSensors(float *P_kPa, float *T_C){
	uint8_t rv = TWI_MasterRead(&twieMaster, METALLUX_ADDRESS, 4); 

	uint16_t Praw = (twieMaster.readData[0]<<8) | twieMaster.readData[1];
	uint16_t Traw = (twieMaster.readData[2]<<8) | twieMaster.readData[3];

	(*P_kPa) = (((float)Praw/32767.)*((Pmax-Pmin)/(Bp-Ap))+(Pmin-((Pmax-Pmin)/(Bp-Ap))*Ap))*100;
	(*T_C) = ((float)Traw/32767.)*((Tmax-Tmin)/(Bt-At))+(Tmin-((Tmax-Tmin)/(Bt-At))*At);

	return twieMaster.bytesRead;

}
uint8_t matchSID(uint8_t SID){
	for (uint8_t iSID = 0; iSID<DEBUG_NVAL; iSID++){
		if(debug_sid[iSID] == SID) return iSID;
	}
	return -1;
}

void collectSensorData(void){

	float P_MPL = null_value; 
	float T_MPL = null_value; 
	uint8_t rv = MPL3115A2_readSensors(&P_MPL, &T_MPL);

	// float P_Metallux = null_value; 
	// float T_Metallux = null_value; 
	// uint8_t rv = Metallux_readSensors(&P_Metallux, &T_Metallux);
	
	data_values[0] = P_MPL;
	data_values[1] = T_MPL;

	data_ready = 1; 
}

// Our slave is always on port C
void TWIC_SlaveProcessData(void)
{

	uint8_t bufIndex = twicSlave.bytesReceived;
	command = twicSlave.receivedData[0];
	if (bufIndex>0) command_argument = twicSlave.receivedData[1];

	switch (command){
		case 0x10:
			// data ready query
			twicSlave.sendData[0] = data_ready;
			break;
		case 0x20:
			// send sensor value, regular or debug
			iSID = matchSID(command_argument);
			if (iSID>=0){
				return_datum = &data_values[iSID];			
			} else {
				return_datum = &null_value;
			}
			memcpy((void *)twicSlave.sendData, return_datum, sizeof(float));	
			data_ready = 0; 		
			break;
		case 0x30:
			// send sensor nval
			twicSlave.sendData[0] = SENSOR_NVAL;
			break;
		case 0x40:
			// send debug nval
			twicSlave.sendData[0] = DEBUG_NVAL;
			break;	
		case 0x50:
			// send sensor sid array
			memcpy((void *)twicSlave.sendData, sensor_sid, SENSOR_NVAL);
			break;	
		case 0x60:
			// send debug sid array
			memcpy((void *)twicSlave.sendData, debug_sid, DEBUG_NVAL);
			break;	
		case 0x70:
			// send data type (== number of bytes)
			twicSlave.sendData[0] = DATA_BYTES;
			break;
		case 0x80:
			// trigger new set of measurements
			collectSensorData();
			break;

	}
}


int main(void)
{
	// the setup
	data_ready = 0;
	for (uint8_t i=0; i<DEBUG_NVAL; i++){
		data_values[i] = null_value;
	}
	
	// When daughter talks to chips on board, it is port E
	// When daughter talks to arduino as slave, it is port C
	/* Initialize TWI master. */
	TWI_MasterInit(&twieMaster,
	              &TWIE,
	              TWI_MASTER_INTLVL_LO_gc,
	              TWI_BAUDSETTING);

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twicSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twicSlave,
	                          SLAVE_ADDRESS,
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	// collect measurements as master
	collectSensorData();

	// the loop
	while (1) {

	}
}

/*! TWIE Master Interrupt vector. */
ISR(TWIE_TWIM_vect){
	TWI_MasterInterruptHandler(&twieMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect){
	TWI_SlaveInterruptHandler(&twicSlave);
}
