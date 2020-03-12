#include "in4073.h"

bool write_log(){

	uint8_t size = 37;
	
	if (count_flash ==0)
	    flash_chip_erase();

	count_flash++;
	uint8_t data[size];
	
	uint32_t time = get_time_us();
	uint8_t sign = 0;
    sign |= (P_bar<0) << 5;
    sign |= (-roll_angle<0) << 4;
    sign |= (pitch_angle<0) <<3;
    sign |= (-roll_rate<0) <<2;
    sign |= (-pitch_rate<0) <<1;
    sign |= (-yaw_rate<0) <<0;

	data[0] = HEADER;
	data[1] = 0x04;
	data[2] = mode_set;
	data[3] = (time>>24) & 0xFF;
	data[4] = (time>>16) & 0xFF;
	data[5] = (time>>8) & 0xFF;
	data[6] = time & 0xFF;
	data[7] = yaw_set;
	data[8] = pitch_set;
	data[9] = roll_set;
	data[10] = lift_set;
	data[11] = sign;
	data[12] = (abs(roll_rate)>>8) & 0xFF;
	data[13] = abs(roll_rate) & 0xFF;
	data[14] = (abs(pitch_rate)>>8) & 0xFF;
	data[15] = abs(pitch_rate) & 0xFF;
	data[16] = (abs(yaw_rate)>>8) & 0xFF;
	data[17] = abs(yaw_rate) & 0xFF;
	data[18] = (abs(roll_angle)>>8) & 0xFF;
	data[19] = abs(roll_angle) & 0xFF;
	data[20] = (abs(pitch_angle)>>8) & 0xFF;
	data[21] = abs(pitch_angle) & 0xFF;
	data[22] = (abs(P_bar)>>24) & 0xFF;
    data[23] = (abs(P_bar)>>16) & 0xFF;
    data[24] = (abs(P_bar)>>8) & 0xFF;
    data[25] = abs(P_bar) & 0xFF;
    data[26] = (uint8_t)(ae[0]>>8) & 0xFF;
	data[27] = (uint8_t) ae[0];
	data[28] = (uint8_t)(ae[1]>>8) & 0xFF;
	data[29] = (uint8_t) ae[1];
	data[30] = (uint8_t)(ae[2]>>8) & 0xFF;
	data[31] = (uint8_t) ae[2];
	data[32] = (uint8_t)(ae[3]>>8) & 0xFF;
	data[33] = (uint8_t) ae[3];
    data[34] = (uint8_t)(bat_volt>>8) & 0xFF;
	data[35] = (uint8_t) bat_volt & 0xFF;
    data[36] = create_crc(data,size-1);

    if (write_pata +size < Max_Flash_address){
        if (flash_write_bytes(write_pata, data, size)){
	        write_pata = write_pata + size;
	    }
        return true;
    }
    else {
        flash_chip_erase();
        write_pata = 0;
        return false;
    }
}

bool erase_log() {
	if(flash_chip_erase())
	    return true;
	else
	    return false;

}
