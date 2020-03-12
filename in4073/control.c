
#include "in4073.h"

/*------------------------------------------------------------------
 * Set Motor Values Function
 *------------------------------------------------------------------
 */

void update_motors(void){

    for(int i=0; i<4; i++){
        if(ae[i]>MAX_SPEED) motor[i] = MAX_SPEED;
		else motor[i] = ae[i];
	}	
}

/*------------------------------------------------------------------
 * Yaw Control Computations
 *------------------------------------------------------------------
 */

void run_yaw_control(){



    //yaw rate control loop

    yaw_rate = sr - sr0;                           // Offset
    yaw_error = -((yaw_set-127)<<8)/15 - yaw_rate; // Divide the setpoint by 15 to limit the yaw control range

    N_needed = P * yaw_error;
    N_needed = N_needed>>11;                       // Shift back to a usable value

    if(N_needed < -127) N_needed = -127;
    if(N_needed > 127) N_needed = 127;

    yaw_ufb = (uint8_t) N_needed + 127;
    set_yaw(yaw_ufb);
}



/*------------------------------------------------------------------
 * Yaw-Pitch-Roll Control Computations
 *------------------------------------------------------------------
 */


void run_filters_and_control() {

    // Yaw Rate Control Loop
    run_yaw_control();

    // Roll Control Loop

    int32_t roll_angle_error;

    roll_angle = phi - phi0;
    roll_angle_error = ((roll_set - 127) << 8) / 6 + roll_angle; // Setpoint is max +-30 degrees (=180/6)

    roll_gain = P1 * roll_angle_error;

    // Roll rate Control Loop
    roll_rate = sp - sp0;                      // Offset
    roll_rate_error = P2 * roll_rate;          // Use of P here allows for independant tuning
    roll_error = roll_gain + roll_rate_error;
    M_needed = roll_error >> 13;               // Shove the result back to usable motor values

    if (M_needed < -127) M_needed = -127;
    if (M_needed > 127) M_needed = 127;

    roll_ufb = (uint8_t) M_needed + 127;
    set_roll(roll_ufb);


    // Pitch Control Loop

    int32_t pitch_angle_error, pitch_gain, pitch_rate_error;

    pitch_angle = theta - theta0;
    pitch_angle_error = ((pitch_set - 127) << 8) / 6 - pitch_angle; // Setpoint is max +-30 degrees (=180/6)
    pitch_gain = P1 * pitch_angle_error;


    // Pitch Rate Control Loop
    pitch_rate = sq - sq0;                        // Offset
    pitch_rate_error = P2 * pitch_rate;;          // Use of P here allows for independant tuning
    pitch_error = pitch_gain + pitch_rate_error;
    L_needed = pitch_error >> 13;                 // Shove the result back to usable motor values

    if (L_needed < -127) L_needed = -127;
    if (L_needed > 127) L_needed = 127;

    pitch_ufb = (uint8_t) L_needed + 127;
    set_pitch(pitch_ufb);
}

/*------------------------------------------------------------------
 * Height Control Computations
 *------------------------------------------------------------------
 */

