/*
    example woth board no.1.

    This board has the NO2 and the Temp-Hum-Press sensors on it.

    Read sensors and print them to Serial Screen
*/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBee868LP.h>
#include <WaspGPS.h>
#include "includes.h"

#define PDEBUG 1   /* enable serial print for debug reasons */
/*#define MODE2  1*/    /* mode 2, when enabled, will read all sensors */

int daytime_mode;

void setup()
{
  #ifdef PDEBUG
  USB.ON();   /* configure the USB port */
  USB.println(F("Board is starting..."));
  #endif

  /* Set the Waspmote ID */
  frame.setID(node_ID);

  /* setup the sensors that will be used in this application */
  setup_app_sensors();

  /* setup RTC with the actual time, from the GPS */
  setup_rtc();

  /* setup the SD card */
  setup_sd_card();
  
  #ifdef PDEBUG
  USB.println("starting application now");
  #endif
}


void loop()
{
  PWR.sleep(WTD_8S, ALL_OFF); /* sleep for 8 seconds, kill all sensors and boards */

  #ifdef PDEBUG
  USB.ON();
  #endif
  
  if( intFlag & WTD_INT ) /* when the 8 seconds of sleep pass, wake up */
  {
    #ifdef PDEBUG
    USB.println(F("---------------------"));
    USB.println(F("WTD INT captured"));
    USB.println(F("---------------------"));
    #endif
    
    intFlag &= ~(WTD_INT); /* when you wake up, clean the flag */
  }

  timer_unit += 1;  /* update the timer unit every time you wake up */

  daytime_mode = strcmp(daytime,"DAY"); /* is it day or night? */

  if(daytime_mode == 0) /* if it is day , the sampling will occur every 6 minutes*/
  {
    if (timer_unit == sample_time_day) /* have we got 45 8_Sec interrupts (360 secs)? */
    {
      timer_unit = 0; /* reset the timer unit */

      /* read the battery level */
      power_level = PWR.getBatteryLevel();

      /* run the application only if there is sufficient power ( > 15% ) */
      if(power_level >= 15)
      {
        run_application(); /* main routine */
      }
    }
  }
  else /* else, if it is night, the sampling will occur every 1 hour */
  {
    if (timer_unit == sample_time_night) /* have we got 45 8_Sec interrupts (360 secs)? */
    {
      timer_unit = 0; /* reset the timer unit */

      /* read the battery level */
      power_level = PWR.getBatteryLevel();

      /* run the application only if there is sufficient power ( > 15% ) */
      if(power_level >= 15)
      {
        run_application(); /* main routine */
      }
    }
  }
  
}

/******************************* FUNCTIONS *******************************/

/*
 * 
 */
void Read_Sensors(void)
{
  /***********************************************/
  /* READ TEMPERATURE, HUMIDITY, AND PRESSURE    */
  #ifdef MODE2
  temperature = Gases.getTemperature();
  delay(100);
  humidity = Gases.getHumidity();
  delay(100);
  pressure = Gases.getPressure();
  delay(100);
  #endif
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

/*
 * 
 */
void Print_Sensors_Values(void)
{
  #ifdef MODE2
  USB.print(F(" Temperature: "));
  USB.print(temperature);
  USB.println(F(" Celsius Degrees |"));

  USB.print(F(" Humidity : "));
  USB.print(humidity);
  USB.println(F(" %RH"));

  USB.print(F(" Pressure : "));
  USB.print(pressure);
  USB.println(F(" Pa"));

  //#ifdef MODE2 /* if we work on full demo, print CO information */
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

/*
 * 
 */
void Create_Ascii(void)
{
  /* power up GPS */
  GPS.ON();

  status = GPS.waitForSignal(TIMEOUT);
  delay(100);

  
  /* Create new frame (ASCII) */
  frame.createFrame(ASCII, node_ID);
  
  /* Add temperature */
  #ifdef MODE2
  frame.addSensor(SENSOR_GASES_TC, temperature);
  /* Add humidity */
  frame.addSensor(SENSOR_GASES_HUM, humidity);
  /* Add pressure */
  frame.addSensor(SENSOR_GASES_PRES, pressure);
  #endif
  
  /* Add CO2 PPM value */
  frame.addSensor(SENSOR_GASES_CO, COPPM);

  #ifdef MODE2 /* if we work on full demo, add other sensors to the frame */
  /* Add CO PPM value */
  frame.addSensor(SENSOR_GASES_CO2, CO2PPM);
  /* Add CO PPM value */
  frame.addSensor(SENSOR_GASES_NO2, NO2PPM);
  #endif

  // add latitude and longtitude
  frame.addSensor(SENSOR_GPS, 
                    GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator),
                    GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator) );

  //Add altitude [m]
  frame.addSensor(SENSOR_ALTITUDE,GPS.altitude);
    
  //Add speed [km/h]
  frame.addSensor(SENSOR_SPEED,GPS.speed);
    
  //Add course [degrees]
  frame.addSensor(SENSOR_COURSE,GPS.course);

  /* turn off the GPS to save power */
  GPS.OFF();

  
  /* add timestamp from RTC */
  RTC.getTime();
  delay(10);
  frame.addTimestamp();

  
  /* Show the frame if we are in debuf mode*/
  #ifdef PDEBUG
  frame.showFrame();
  #endif


  /* save the frame to the SD card */
  memset(toWrite, 0x00, sizeof(toWrite) );

  // Conversion from Binary to ASCII
  Utils.hex2str( frame.buffer, toWrite, frame.length);
  
  /* some debug help here */
  #ifdef PDEBUG
  USB.print(F("Frame to be stored:"));
  USB.println(toWrite);
  #endif

  /* now append data to file */
  sd_answer = SD.appendln(filename, toWrite);
  
  if( sd_answer == 1 )
  {
    #ifdef PDEBUG
    USB.println(F("Frame appended to file"));
    #endif
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("Append failed"));
    #endif
  }
  
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

/*
 * 
 */
void read_logged_data_and_send(void)
{
  /* init XBee */
  xbee868LP.ON();
  
  // get number of lines in file
  numLines = SD.numln(filename);

  // get specified lines from file
  // get only the last file line
  startLine = numLines - 1; 
  endLine = numLines;

  // iterate to get the File lines specified
  for( int i = startLine; i < endLine ; i++ )
  {  
    // Get 'i' line -> SD.buffer
    SD.catln( filename, i, 1); 
    
    // initialize frameSD
    memset(frameSD, 0x00, sizeof(frameSD) ); 
    
    // conversion from ASCII to Binary 
    lengthSD = Utils.str2hex(SD.buffer, frameSD );


    /* not actually needed here -- just for debug */
    // Conversion from ASCII to Binary
    #ifdef PDEBUG
    USB.print(F("Get previously stored frame:"));
    #endif
    
    for(int j = 0; j < lengthSD; j++)
    { 
      #ifdef PDEBUG   
      USB.print(frameSD[j],BYTE);
      #endif
    }
    #ifdef PDEBUG
    USB.println();
    #endif
    
    /************************************************
    * At this point 'frameSD' and 'lengthSD' can be 
    * used as 'frame.buffer' and 'frame.length' to 
    * send information via some communication module 
    *************************************************/
  }

  #ifdef PDEBUG
  USB.println();
  USB.println();
  #endif
  /* debug ends here */

  /* here send the frame via XBEE */
  /* then, send the frame */
  error = xbee868LP.send( MESHLIUM_ADDRESS, frameSD, lengthSD );

  // check TX flag
  if( error == 0 )
  {
    #ifdef PDEBUG
    USB.println(F("send ok"));
    #endif
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("send error"));
    #endif
  }

  /* turn ZigBee off, to save power */
  xbee868LP.OFF();
}

/*
 * 
 */
void setup_sd_card(void)
{
  // Set SD ON, to log samples
  SD.ON();

  // Delete file if already exists, to discard old samples
  sd_answer = SD.del(filename);

  if(sd_answer) 
  {
    #ifdef PDEBUG
    USB.println(F("file deleted"));
    #endif
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("file NOT deleted"));
    #endif
  }

  // create new file
  sd_answer = SD.create(filename);

  if(sd_answer)
  { 
    #ifdef PDEBUG
    USB.println(F("file created"));
    #endif
  }
  else
  {
    #ifdef PDEBUG
    USB.println(F("file not created"));
    #endif
  }
}

/*
 * 
 */
void setup_rtc(void)
{
  /* Enable Real Time Clock */
  RTC.ON();

  // set GPS ON  
  GPS.ON();

  /* wait for GPS signal for specific time */
  status = GPS.waitForSignal(TIMEOUT);

  /* if GPS is connected then set Time and Date to RTC */ 
  if( status == true )
  {    
    // set time in RTC from GPS time (GMT time)
    GPS.setTimeFromGPS();
  }

  /* switch GPS off, to save power */
  GPS.OFF();
}

/*
 * 
 */
void setup_app_sensors(void)
{
  /* Calculate the slope and the intersection of the logarithmic functions */
  COSensor.setCalibrationPoints(COres, COconcentrations, numPoints);           /* for CO  sensor */
  
  #ifdef MODE2 /* if we work on full demo, load other sensors */
  CO2Sensor.setCalibrationPoints(CO2voltages, CO2concentrations, numPoints);   /* for CO2 sensor */
  
  NO2Sensor.setCalibrationPoints(NO2res, NO2concentrations, numPoints);        /* for NO2 sensor */
  #endif
}

/*
 * 
 */
bool check_the_date(void)
{
  /* see the date */
  char* Date = strtok(RTC.getTime(),":");

  /* check if it is day or night */
  snprintf( daytime, sizeof(daytime), "%s", "DAY");

  /* if one day has passed, return TRUE */
  if(Date)
  {
    return true;
  }
  /* else return FALSE */
  else
  {
    return false;
  }
}

/*
 * 
 */
void run_application(void)
{
    /* enable sensors */
    Gases.ON();

    /* enable the CO sensor */
    COSensor.ON();

    #ifdef MODE2 /* if we work on full demo, start other sensors */
    CO2Sensor.ON();
    NO2Sensor.ON();
    #endif

    /* wait 1 minute for the sensors to warm-up */
    one_minute_delay();

    
    /**** READ SENSORS ****/
    Read_Sensors();

    #ifdef PDEBUG /* only print to serial if the debug mode is set */
    /**** PRINT OF THE RESULTS ****/
    Print_Sensors_Values();
    #endif

    /**** CREATE ASCII FRAME ****/
    Create_Ascii();

    /* if a day has passed, send the logged data to Meshlium */
    if(check_the_date() == true)
    {
      read_logged_data_and_send();
    }
}

/*
 * 
 */
void one_minute_delay(void)
{
  uint8_t i = 0; /* 60 seconds counter */

  for(i = 0; i < 60; i++)
  {
    delay(1000);
  }
}
