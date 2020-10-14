#include "UM7.h"

#define DREG_EULER_PHI_THETA 0x70	// Packet address sent from the UM7 that contains roll,pitch,yaw and rates.
#define DREG_GYRO_PROC_X 0x61
#define DREG_GYRO_RAW_XY 0x56 
#define SOMETHING_WEIRD 0x89

UM7::UM7() : state(STATE_ZERO){}	// Default constructor



bool UM7::encode(byte c){
	// Serial.print("byte: "); 
        if (c == 's' || c == 'n' || c == 'p') {
        // Serial.print(" HEX ");
        // Serial.println(c, HEX);
}

        // Serial.println(c, BIN);
	switch(state){
	case STATE_ZERO:
		if (c == 's'){
			state = STATE_S;		// Entering state S from state Zero
		} else {
			state = STATE_ZERO;
		}
// Serial.println("State zero");
		return false;
	case STATE_S:
		if (c == 'n'){
			state = STATE_SN;		// Entering state SN from state S
		} else {
			state = STATE_ZERO;
		}
// Serial.println("State s");
		return false;
	case STATE_SN:
		if (c == 'p'){
			state = STATE_SNP;		// Entering state SNP from state SN.  Packet header detected.
		} else {
			state = STATE_ZERO;
		}
// Serial.println("State sn");
		return false;
	case STATE_SNP:
		state = STATE_PT;			// Entering state PT from state SNP.  Decode packet type.
		packet_type = c;
		packet_has_data = (packet_type >> 7) & 0x01;
//roll = int16_t(packet_has_data);

		packet_is_batch = (packet_type >> 6) & 0x01;
//yaw = int16_t(packet_is_batch);
		batch_length    = (packet_type >> 2) & 0x0F;
// Serial.print("batch length "); 
// Serial.println(batch_length);
// Serial.send_now();
		if (packet_has_data){
			if (packet_is_batch){
				data_length = 4 * batch_length;	// Each data packet is 4 bytes long
			} else {
				data_length = 4;
			}
		} else {
			data_length = 0;
		}  
// roll_rate = int16_t(data_length);
// Serial.print("data length "); 
// Serial.println( roll_rate);

// Serial.println("state snp");
// Serial.send_now();
		return false;
	case STATE_PT:
		state = STATE_DATA;		// Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
		address = c;
		data_index = 0;
// Serial.print("Adress ");
// Serial.println(address, HEX);
// Serial.print("has data ");
// Serial.println(packet_has_data);
// Serial.print("is batch " );
// Serial.println(packet_is_batch);
//pitch = int16_t(address);
// Serial.println("state pt");
// Serial.print("data index "); 
// Serial.println(data_index); 

		return false;
	case STATE_DATA:			//  Entering state READ_DATA.  Stay in state until all data is read.
// Serial.println("state data");
		data[data_index] = c;
// // Serial.print("data index ");
// // Serial.println(data_index);
// Serial.println(c, DEC);

		data_index++;
// Serial.print("data index ");

// Serial.println(data_index);
// Serial.print("data length ");
// Serial.println(data_length);
		if (data_index >= data_length){
// Serial.println("returning state zero");
// Serial.send_now();
			state = STATE_CHK1;	//  Data read completed.  Next state will be CHK1
		}

		return false;
	case STATE_CHK1:			// Entering state CHK1.  Next state will be CHK0
		state = STATE_CHK0;
// Serial.println(c, HEX);
		checksum1 = c;
		return false;
	case STATE_CHK0: 				
		state = STATE_ZERO;		// Entering state CHK0.  Next state will be state Zero.
		checksum0 = c;
// Serial.println(c, HEX);
		return checksum();
	}
}

bool UM7::checksum(){
	checksum10  = checksum1 << 8;	// Combine checksum1 and checksum0
	checksum10 |= checksum0;
	computed_checksum = 's' + 'n' + 'p' + packet_type + address;
// Serial.print("data length ");
// Serial.println(data_length, DEC);
	for (int i = 0; i < data_length; i++){
		computed_checksum += data[i];
	}
//       Serial.print("checksum computed ");
// Serial.println(computed_checksum, HEX);
//  Serial.print("checksum received ");
// Serial.println(checksum10, HEX); 
	if (checksum10 == computed_checksum){
// Serial.println("CHecksum great"); 
		save();
		return true;
	} else {
		return false;
	}
}

void UM7::save(){
// Serial.println("Preprocessing");
// Serial.println(address, HEX);
/* This was used for debugging the stuff 
        Serial.print("Adress ----: ");
        Serial.println(address, HEX);
        Serial.print("Data length: ");
        Serial.println(data_length, DEC);
        Serial.print("Packet Type: ");
        Serial.println(packet_type, BIN);
        Serial.send_now(); */
	switch(address){
	case DREG_EULER_PHI_THETA :		// data[6] and data[7] are unused.
		if(packet_is_batch){
			roll = data[0] << 8;
			roll |= data[1];
			pitch = data[2] << 8;
			pitch |= data[3];
			yaw = data[4] << 8;
			yaw |= data[5];
			roll_rate = data[8] << 8;
			roll_rate |= data[9];
			pitch_rate = data[10] << 8;
			pitch_rate |= data[11];
			yaw_rate = data[12] << 8;
			yaw_rate |= data[13];
		}else{
			roll = data[0] << 8;
			roll |= data[1];
			pitch = data[2] << 8;
			pitch |= data[3];
		}
		break;
        case DREG_GYRO_PROC_X : 
                if(packet_is_batch) {
// Serial.println("Processing");
//                         for(int i = 0; i < 4; i++) angular_rate_x.b[i] = data[i];
// Serial.println(data[0]);Serial.println(data[1]);Serial.println(data[2]);Serial.println(data[3]);
                 }
             break;
       case DREG_GYRO_RAW_XY : 
                if(packet_is_batch) {
                    // Serial.println("Got to 0x56 and batch");
                    // Serial.send_now();
                    Gyro_RAW_X = data[0] << 8;
                    Gyro_RAW_X |= data[1];
                    Gyro_RAW_Y = data[2] << 8;
                    Gyro_RAW_Y |= data[3];
                    Gyro_RAW_Z = data[4] << 8;
                    Gyro_RAW_Z |= data[5];
                    res_gyro_raw = data[6] << 8;
                    res_gyro_raw |= data[7];
                    for(int i = 0; i < 4; i++)
                      encodefloat.byte_number[3-i] = data[8+i];
                    Gyro_RAW_Time = encodefloat.float_number;
                    Accel_RAW_X = data[12] << 8;
                    Accel_RAW_X |= data[13];
                    Accel_RAW_Y = data[14] << 8;
                    Accel_RAW_Y |= data[15];
                    Accel_RAW_Z = data[16] << 8;
                    Accel_RAW_Z |= data[17];
                    res_acc_raw = data[18] << 8;
                    res_acc_raw |= data[19];
                    for(int i = 0; i < 4; i++)
                      encodefloat.byte_number[3-i] = data[20+i];
                    Accel_RAW_Time = encodefloat.float_number;
                    Mag_RAW_X = data[24] << 8;
                    Mag_RAW_X |= data[25];
                    Mag_RAW_Y = data[26] << 8;
                    Mag_RAW_Y |= data[27];
                    Mag_RAW_Z = data[28] << 8;
                    Mag_RAW_Z |= data[29];
                    res_mag_raw = data[30] << 8;
                    res_mag_raw |= data[31];
                    for(int i = 0; i < 4; i++)
                      encodefloat.byte_number[3-i] = data[32+i];
                    Mag_RAW_Time = encodefloat.float_number;
                    for(int i = 0; i < 4; i++)
                      encodefloat.byte_number[3-i] = data[36+i];
                    Temperature_degrees_C = encodefloat.float_number;
                    for(int i = 0; i < 4; i++)
                      encodefloat.byte_number[3-i] = data[40+i];
                    Temperature_Time = encodefloat.float_number;
             }
            break; 
	}
}
