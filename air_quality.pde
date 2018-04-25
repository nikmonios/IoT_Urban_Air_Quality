/*
    example woth board no.1.

    This board has the NO2 and the Luminosity sensors on it.

    Read sensors and print them to Serial Screen
*/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBeeZB.h>
#include "coefficients.h"

#define PDEBUG 1    /* enable serial print for debug reasons */
/*#define MODE2  1*/    /* mode 2, when enabled, will read all sensors */ 

void setup()
{
  #ifdef PDEBUG
  USB.ON();   /* configure the USB port */
  USB.println(F("Board is starting..."));
  #endif

  
  /* Calculate the slope and the intersection of the logarithmic functions */
  COSensor.setCalibrationPoints(COres, COconcentrations, numPoints);           /* for CO  sensor */
  
  #ifdef MODE2 /* if we work on full demo, load other sensors */
  CO2Sensor.setCalibrationPoints(CO2voltages, CO2concentrations, numPoints);   /* for CO2 sensor */
  NO2Sensor.setCalibrationPoints(NO2res, NO2concentrations, numPoints);        /* for NO2 sensor */
  #endif

  /* Set the Waspmote ID */
  frame.setID(node_ID);

  /* Enable Real Time Clock */
  RTC.ON();

  /* Check if there is a Meshlium nearby */
  error = xbeeZB.setRTCfromMeshlium(MESHLIUM_ADDRESS);
  
  if ( error == 0) /* if we have a Meshlium nearby */
  {
    /* set real time and date as taken from the Meshlium */
    RTC.getTime();
    delay(10);
    
    #ifdef PDEBUG
    USB.println("took real time and date from Meshlium");
    #endif
  }

  #ifdef PDEBUG
  USB.println("starting application now");
  #endif
}


void loop()
{
  PWR.sleep(WTD_4S, ALL_OFF); /* sleep for 4 seconds, kill all sensors and boards */

  #ifdef PDEBUG
  USB.ON();
  #endif
  
  if( intFlag & WTD_INT ) /* when the 4 seconds of sleep pass, wake up */
  {
    #ifdef PDEBUG
    USB.println(F("---------------------"));
    USB.println(F("WTD INT captured"));
    USB.println(F("---------------------"));
    #endif
    
    intFlag &= ~(WTD_INT); /* when you wake up, clean the flag */
  }

  timer_unit += 1;  /* update the timer unit every time you wake up */

  if (timer_unit == 3) /* have we got 3 4_Sec interrupts (12 secs)? */
  {
    timer_unit = 0; /* reset the timer unit */

    /* init XBee */
    xbeeZB.ON();

    /* and enable sensors */
    Gases.ON();
    COSensor.ON();

    #ifdef MODE2 /* if we work on full demo, start other sensors */
    CO2Sensor.ON();
    NO2Sensor.ON();
    #endif
    
    delay(100);

    /**** READ SENSORS ****/
    Read_Sensors();

    #ifdef PDEBUG /* only print to serial if the debug mode is set */
    /**** PRINT OF THE RESULTS ****/
    Print_Sensors_Values();
    #endif

    /**** CREATE ASCII FRAME ****/
    Create_Ascii();
  }
}

/**** FUNCTIONS ****/

void Read_Sensors()
{
  /***********************************************/
  /* READ TEMPERATURE, HUMIDITY, AND PRESSURE    */
  delay(4000);
  temperature = Gases.getTemperature();
  delay(100);
  humidity = Gases.getHumidity();
  delay(100);
  pressure = Gases.getPressure();
  delay(100);
  /**********************************************/
  
  /*                READ CO                       */
  COPPM = COSensor.readConcentration();
  /***********************************************/


  #ifdef MODE2 /* if we work on full demo, read other sensors */
  /*                READ CO2                      */
  CO2PPM = CO2Sensor.readConcentration();
  /**********************************************/
  
  /*                READ NO2                      */
  NO2PPM = NO2Sensor.readConcentration();
  #endif
}


void Print_Sensors_Values()
{
  USB.print(F(" Temperature: "));
  USB.print(temperature);
  USB.println(F(" Celsius Degrees |"));

  USB.print(F(" Humidity : "));
  USB.print(humidity);
  USB.println(F(" %RH"));

  USB.print(F(" Pressure : "));
  USB.print(pressure);
  USB.println(F(" Pa"));

  #ifdef MODE2 /* if we work on full demo, print CO information */
  USB.print(F(" CO2 concentration estimated: "));
  USB.print(CO2PPM);
  USB.println(F(" ppm"));
  #endif

  USB.print(F(" CO concentration Estimated: "));
  USB.print(COPPM);
  USB.println(F(" ppm"));

  #ifdef MODE2 /* if we work on full demo, print NO2 information */
  USB.print(F(" NO2 concentration Estimated: "));
  USB.print(NO2PPM);
  USB.println(F(" ppm"));a
  #endif
}

void Create_Ascii()
{
  /* first see if we have achieved communication with Meshlium */
  error = xbeeZB.setRTCfromMeshlium(MESHLIUM_ADDRESS);

  /* Create new frame (ASCII) */
  frame.createFrame(ASCII, node_ID);
  /* Add temperature */
  frame.addSensor(SENSOR_GASES_TC, temperature);
  /* Add humidity */
  frame.addSensor(SENSOR_GASES_HUM, humidity);
  /* Add pressure */
  frame.addSensor(SENSOR_GASES_PRES, pressure);
  /* Add CO2 PPM value */
  frame.addSensor(SENSOR_GASES_CO, COPPM);

  #ifdef MODE2 /* if we work on full demo, add other sensors to the frame */
  /* Add CO PPM value */
  frame.addSensor(SENSOR_GASES_CO2, CO2PPM);
  /* Add CO PPM value */
  frame.addSensor(SENSOR_GASES_NO2, NO2PPM);
  #endif

  if ( error == 0) /* if we have the time from the Meshlium */
  {
    /* add timestamp from RTC */
    RTC.getTime();
    delay(10);
    frame.addTimestamp();
  }
  /* Show the frame */
  frame.showFrame();

  /* reset values */
  temperature = 0;
  humidity = 0;
  pressure = 0;
  COPPM = 0;
  #ifdef MODE2 /* if we work on full demo, clear other sensors */
  CO2PPM = 0;
  NO2PPM = 0;
  #endif
}
