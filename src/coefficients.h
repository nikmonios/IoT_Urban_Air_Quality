#ifndef COEFFICIENTS_H
#define COEFFICIENTS_H


/* Concentratios used in calibration process (PPM Values) */
#define POINT1_PPM_CO2 350.0  //   <-- Normal concentration in air
#define POINT2_PPM_CO2 1000.0
#define POINT3_PPM_CO2 3000.0

#define POINT1_PPM_NO2 10.0   // <-- Normal concentration in air
#define POINT2_PPM_NO2 50.0   
#define POINT3_PPM_NO2 100.0 

/* Calibration vVoltages obtained during calibration process (Volts) */
#define POINT1_VOLT_CO2 0.300
#define POINT2_VOLT_CO2 0.350
#define POINT3_VOLT_CO2 0.380

#define POINT1_RES_NO2 45.25  // <-- Rs at normal concentration in air
#define POINT2_RES_NO2 25.50
#define POINT3_RES_NO2 3.55

#define POINT1_PPM_CO 100.0   // <--- Ro value at this concentration
#define POINT2_PPM_CO 300.0   // 
#define POINT3_PPM_CO 1000.0  // 

/* Calibration resistances obtained during calibration process */
#define POINT1_RES_CO 230.30 // <-- Ro Resistance at 100 ppm. Necessary value.
#define POINT2_RES_CO 40.665 //
#define POINT3_RES_CO 20.300 //

/* Define the number of calibration points */
#define numPoints 3

/**** GLOBAL & DEFINES ****/
float NO2PPM = 0;    /* Stores the NO2 value */
float COPPM = 0;    /* Stores the CO value */
float temperature = 0; /* Stores the temperature in ºC */
float humidity = 0;     /* Stores the realitve humidity in %RH */
float pressure = 0;    /* Stores the pressure in Pa */
float CO2PPM = 0; /* stores the CO2 value */


/* CO2 Sensor must be connected physically in SOCKET_2 */
CO2SensorClass CO2Sensor;
/* NO2 Sensor must be connected physically in SOCKET_3 */
NO2SensorClass NO2Sensor; 
/* CO Sensor must be connected physically in SOCKET_4 */
COSensorClass COSensor;

float COconcentrations[] =   { POINT1_PPM_CO, POINT2_PPM_CO, POINT3_PPM_CO };
float COres[] =              { POINT1_RES_CO, POINT2_RES_CO, POINT3_RES_CO };
float NO2concentrations[] =  { POINT1_PPM_NO2, POINT2_PPM_NO2, POINT3_PPM_NO2 };
float NO2res[] =             { POINT1_RES_NO2, POINT2_RES_NO2, POINT3_RES_NO2 };
float CO2concentrations[] =  { POINT1_PPM_CO2, POINT2_PPM_CO2, POINT3_PPM_CO2 };
float CO2voltages[] =        { POINT1_VOLT_CO2, POINT2_VOLT_CO2, POINT3_VOLT_CO2 };


/* node ID */
char node_ID[] = "node1_data";
char node_gps[] = "node1_gps";


/* a unit that updates every 8 seconds */
uint16_t timer_unit = 0; 

/* 45 * 8 seconds = 360 seconds. Sampling period during the day */
uint16_t sample_time_day = 45; 

/* 450 * 8 seconds = 3600 seconds. Sampling period during the night */
uint16_t sample_time_night = 450;

/* Destination Meshlium MAC address */
//////////////////////////////////////////
char MESHLIUM_ADDRESS[] = "000000000000FFFF"; /* broadcast mode */
//////////////////////////////////////////

/* define variable to check if we can communicate with Meshlium */
uint8_t error;

/* define GPS timeout when connecting to satellites */
/* this time is defined in seconds (240sec = 4minutes) */
#define TIMEOUT 240

/* define status variable for GPS connection */
bool status;

/* variables for data logging */
/* define file name: MUST be 8.3 SHORT FILE NAME */
char filename[]="FILE1.TXT";
/* define file name for GPS coordinates logging */
char filename_gps[] = "GPSDATA.TXT";

char filename_bat[] = "BATLOG.TXT";

/* buffer to write into Sd File */
char toWrite[256];

/* define variables to read stored frames */
uint8_t frameSD[MAX_FRAME + 1];
uint16_t lengthSD;
int32_t numLines;

/* variables to define the file lines to be read */
int startLine;
int endLine;

/* define variable */
uint8_t sd_answer;

/* battery level indicator */
uint8_t power_level;

/* Day or Night? for night mode */
char* daytime = "DAY"; /* initialize in day */

/* temporary day of week variable, to see if a day has passed */
char* temp_day = "Mon";

/* an index that shows which sample inside the .txt file is next to be sent */
int transmission_index = 0;

/* variable to store whether we are operating during the day or during the night */
int daytime_mode;

/* from 8 AM, start taking samples every 6 minutes */
int morning_limit = 8;

/* from 8 PM, start taking samples every one hour */
int night_limit = 20;

#endif