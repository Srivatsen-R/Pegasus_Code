#include "foc_mtpa_lut.h"


mtpa_lut mtpa = {
               .rpm_index=0,
               .torque_index=0,
               .speed=0,
               .torque=0
                };


const int mtpa_lut_id[16][6] = 
{-5.3,	-5.3,	  -5.6,	    -5.3,	     -5.3,	  -5,
-15.19,	-15.5,	  -15.6,	-15.3,	    -14.4,	  -14.4,
-26.11,	-26.3,	  -26.5,	-26.1,	    -25.8,	  -24.7,
-38.41,	-38.5,	  -38.5,	-38.6,	    -38.5,	  -50.9,
-50.21,	-51.5,	  -52,	    -51.2,	    -52.2,	  -80.3,
-62.23,	-64.6,	  -64.8,	-65.9,	    -70.5,	  -113.1,
-77.4,  -78.54,	  -80.06,	-81.53,	    -90.64,	  -138.69,
-92.3,	-93.33,	  -95.36,	-95.83,	    -118.73,  -175.74,
-105.4,	-107.82,  -109.39,	-109.89,	-147.19,  -218.26,
-119.2,	-120.46,  -124.88,	-124.35,	-180.52,   0.00,
-134.6,	-137.09,  -140.70,	-143.27,	-210.78,   0.00,
-150.6,	-153.13,  -158.01,	-173.27,	0,	       0,
-166.6,	-169.52,  -174.61,	-175.38,	0,	       0,
-179.7,	-187.32,  -188.44,	 0,	        0,	       0,
-198.7,	-201.48,  -201.54,	 0,	        0,	       0,
0,     0,          0,        0,         0,         0};



const int mtpa_lut_iq[16][6] = 
{34.18,	34.18,	33.2,	32.1,	31.9,	30.6,
57.6,	56.9,	57.1,	57.2,	56.5,	54.9,
76.46,	76.7,	77.5,	77.1,	76.8,	74.5,
95.67,	95.50,	95.5,	96.6,	95.7,	80.7,
112.6,	113.2,	114.5,	114.2,	114.7,	82.2,
129.48,	130.5,	130.9,	132.9,	127,	79.9,
147.2,	147.72,	150.53,	151.89,	130.87,	85.43,
163.5,	165.9,	167.21,	167.87,	128.7,	84.73,
179.4,	182.54,	183.71,	184.9,	125.75,	84.08,
196.2,	196.87,	201.5,	200.93,	122.38,	0,
212,	215.01,	218.63,	215.16,	122.31,	0,
230.9,	233.21,	237.42,	204.,   0,	    0,
248.3,	249.93,	255.78,	234.83,	0,	    0,
261.38,	269.12,	270.9,	0,	    0,	    0,
281.77,	284.96,	285.04,	0,	    0,	    0,
0,	    0,	    0,	    0,	    0,	    0};



//0 to 15 , 5 to 80nm
//0 to 6  , 500 to 3000rpm


int TORQUE_INDEX_CALCULATE(int torque )
{
if(torque>=0 && torque<5){ }
else if(torque>=0  && torque<10){mtpa.torque_index=0;}
else if(torque>=10 && torque<15){mtpa.torque_index=1;}
else if(torque>=15 && torque<20){mtpa.torque_index=2;}
else if(torque>=20 && torque<25){mtpa.torque_index=3;}
else if(torque>=25 && torque<30){mtpa.torque_index=4;}
else if(torque>=30 && torque<35){mtpa.torque_index=5;}
else if(torque>=35 && torque<40){mtpa.torque_index=6;}
else if(torque>=40 && torque<45){mtpa.torque_index=7;}
else if(torque>=45 && torque<50){mtpa.torque_index=8;}
else if(torque>=50 && torque<55){mtpa.torque_index=9;}
else if(torque>=55 && torque<60){mtpa.torque_index=10;}
else if(torque>=60 && torque<65){mtpa.torque_index=11;}
else if(torque>=65 && torque<70){mtpa.torque_index=12;}
else if(torque>=70 && torque<75){mtpa.torque_index=13;}
else if(torque>=75 && torque<80){mtpa.torque_index=14;}
else if(torque>=80 && torque<85){mtpa.torque_index=15;}

return mtpa.torque_index;
}


int RPM_INDEX_CALCULATE(int rpm)
{
if(rpm>=0 && rpm<500){mtpa.rpm_index=0; }
else if(rpm>=500  && rpm<1000){mtpa.rpm_index=1;}
else if(rpm>=1000  && rpm<1500){mtpa.rpm_index=2;}
else if(rpm>=1500  && rpm<2000){mtpa.rpm_index=3;}
else if(rpm>=2500  && rpm<3000){mtpa.rpm_index=4;}
else if(rpm>=3000  && rpm<3500){mtpa.rpm_index=5;}

return mtpa.rpm_index;
}
