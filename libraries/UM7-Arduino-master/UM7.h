#ifndef UM7_H
#define UM7_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

class UM7{
public:
	short roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
        int16_t Gyro_RAW_X, Gyro_RAW_Y, Gyro_RAW_Z, res_gyro_raw;
        float Gyro_RAW_Time;
        int16_t Accel_RAW_X, Accel_RAW_Y, Accel_RAW_Z, res_acc_raw;
        float Accel_RAW_Time; 	
        int16_t Mag_RAW_X, Mag_RAW_Y, Mag_RAW_Z, res_mag_raw; 
        float Mag_RAW_Time; 
        float Temperature_degrees_C; 
        float Temperature_Time; 
        
	UM7();
	union
        {      byte byte_number[4] ;
        float float_number ;
        } encodefloat ;
	bool encode(byte c);
	
private:

	int state;
	
	enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};
	
	byte packet_type;
	byte address;
	bool packet_is_batch;
	byte batch_length;
	bool packet_has_data;
	byte data[64];
	byte data_length;
	byte data_index;

	byte checksum1;		// First byte of checksum
	byte checksum0;		// Second byte of checksum

	unsigned short checksum10;			// Checksum received from packet
	unsigned short computed_checksum;	// Checksum computed from bytes received
	
	bool checksum(void);
	
	void save(void);
};

#endif
