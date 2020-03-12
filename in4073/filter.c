#include "in4073.h"

// taking Fs = 1000 Hz , Fc = 10Hz , [a,b] = butter(10,500,'low')

int16_t butter_filter(int16_t x)   //x=sr?  Yaw?
{
	static	 int16_t a0 = 15;    //Found through matlab and then fixed point arithmetic --14 bits for decimals
	static	 int16_t a1 = 31;    
	static	 int16_t a2 = 15;   
	//static	 int16_t b0 =  16384;   // commenting since not used  
	static	 int16_t b1 = -31313;
    static   int16_t b2 = 14991;
         int16_t x0 = x; 
		 int16_t x1 = 0;
		 int16_t x2 = 0;
		 int16_t y0 = 0;
		 int16_t y1 = 0;
         int16_t y2 = 0;

        y0 = (    mul(a0, x0)
				+ mul(a1, x1)
				+ mul(a2, x2) 
				- mul(b1, y1) 
				- mul(b2, y2));
		sr = y0;

		//updating values in the array.
		x2 = x1;		
		y2 = y1;
		x1 = x0;
y1 = y0;
return sr;
}
 
int16_t mul(int16_t a, int16_t b)
{
	int32_t result = a * b; 
	int16_t ans = (int16_t)(result >> 14); 
	return ans;
}