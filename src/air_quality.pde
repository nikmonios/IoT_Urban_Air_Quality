/*
    example woth board no.1.

    This board has the NO2 and the Temp-Hum-Press sensors on it.

    Read sensors and print them to Serial Screen
*/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBee868LP.h>
#include <WaspGPS.h>
#include "coefficients.h"

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

  /* setup the SD card */
  setup_sd_card();

  /* setup the GPS and RTC modules */
  setup_gps_and_rtc();
  
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

  timer_unit++;  /* update the timer unit every time you wake up */

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

        /* check if battery is charging from solar panel, in debug mode */
        #ifdef PDEBUG
        see_battery_status();
        #endif
      }
    }
  }
  else /* else, if it is night, the sampling will occur every 1 hour */
  {
    if (timer_unit == sample_time_night) /* have we got 450 8_Sec interrupts (3600 secs)? */
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
  /**************MODE 2 ONLY*********************/
  #ifdef MODE2
  
  /*                READ CO2                    */
  CO2PPM = CO2Sensor.readConcentration();
  /**********************************************/
  
  /*                READ NO2                    */
  NO2PPM = NO2Sensor.readConcentration();
  #endif
  /***************MODE 2 ENDS*********************/


  /* READ TEMPERATURE, HUMIDITY, PRESSURE        */
  temperature = Gases.getTemperature();
  delay(100);
  humidity = Gases.getHumidity();
  delay(100);
  pressure = Gases.getPressure();
  delay(100);
  
  /*                READ CO                      */
  COPPM = COSensor.readConcentration();
  /***********************************************/

}

/*
 * 
 */
void Print_Sensors_Values(void)
{
  /* MODE 2 DEBUG INFO STARTS HERE */
  #ifdef MODE2

  USB.print(F(" NO2 concentration Estimated: "));
  USB.print(NO2PPM);
  USB.println(F(" ppm"));

  USB.print(F(" CO2 concentration estimated: "));
  USB.print(CO2PPM);
  USB.println(F(" ppm"));
  #endif
  /* MODE 2 DEBUG INFO ENDS HERE */

  USB.print(F(" CO concentration Estimated: "));
  USB.print(COPPM);
  USB.println(F(" ppm"));

  USB.print(F(" Temperature: "));
  USB.print(temperature);
  USB.println(F(" Celsius Degrees |"));

  USB.print(F(" Humidity : "));
  USB.print(humidity);
  USB.println(F(" %RH"));

  USB.print(F(" Pressure : "));
  USB.print(pressure);
  USB.println(F(" Pa"));

}

/*
 * 
 */
void Create_Ascii(void)
{
  /* power up SD card */
  SD.ON();
  
  /* power up GPS */
  GPS.ON();

  /* load ephemeris previously stored in SD */
  GPS.loadEphems();
  
  status = GPS.waitForSignal(TIMEOUT);

  if( status == true )
  {
    USB.println("creating new frame now");
    
    /* Create new frame (ASCII) */
    frame.createFrame(BINARY); /* frame.createFrame(ASCII, node_ID) */

    /**************MODE 2 ONLY******************/
    #ifdef MODE2
   
    /* Add CO PPM value */
    frame.addSensor(SENSOR_GASES_CO2, CO2PPM);
    
    /* Add NO2 PPM value */
    frame.addSensor(SENSOR_GASES_NO2, NO2PPM);
    
    #endif
    /**************MODE 2 ENDS******************/
  
    /* Add CO2 PPM value */
    frame.addSensor(SENSOR_GASES_CO, COPPM);

    /* Add temperature */
    frame.addSensor(SENSOR_GASES_TC, temperature);
    
    /* Add humidity */
    frame.addSensor(SENSOR_GASES_HUM, humidity);
    
    /* Add pressure */
    frame.addSensor(SENSOR_GASES_PRES, pressure);

    // add latitude and longtitude
    frame.addSensor(SENSOR_GPS, 
                    GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator),
                    GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator) );

    /* turn off the GPS to save power */
    GPS.OFF();
    
    /* add timestamp from RTC */
    RTC.getTime();
    frame.addTimestamp();
  }
  
  /* Show the frame if we are in debuf mode*/
  #ifdef PDEBUG
  frame.showFrame();
  #endif

  /* save the frame to the SD card */
  memset(toWrite, 0x00, sizeof(toWrite) );

  /* Conversion from Binary to ASCII */
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


  /* close SD */
  SD.OFF();

  
  /* reset values */
  COPPM = 0;
  temperature = 0;
  humidity = 0;
  pressure = 0;
  
  #ifdef MODE2 /* if we work on full demo, clear other sensors */
  CO2PPM = 0;
  NO2PPM = 0;
  #endif
}

/*
 * 
 */
void read_logged_data_and_send(int index)
{
  /* speed variable will check if the vehicle is moving or not */
  int vehicle_speed = 0;
  
  /* init XBee */
  xbee868LP.ON();

  /* open SD card */
  SD.ON();
  
  /* get number of lines in file */
  numLines = SD.numln(filename);

  /* get specified lines from file
     get only the last file line*/
  startLine = index; 
  endLine = numLines;

  /* iterate to get the File lines specified */
  for( int i = startLine; i < endLine ; i++ )
  {  
    /* Get 'i' line -> SD.buffer */
    SD.catln( filename, i, 1); 
    
    /* initialize frameSD */
    memset(frameSD, 0x00, sizeof(frameSD) ); 
    
    /* conversion from ASCII to Binary */
    lengthSD = Utils.str2hex(SD.buffer, frameSD );


    /* not actually needed here -- just for debug */
    /* Conversion from ASCII to Binary */
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

    vehicle_speed = (int)GPS.speed;
  
    if (vehicle_speed < 5) /* vehicle has slowed down or not moving */
    {
      /* here send the frame via XBEE */
      /* send the frame to anyone listening */
      error = xbee868LP.send( MESHLIUM_ADDRESS, frameSD, lengthSD );

      // check TX flag
      if( error == 0 )
      {
        transmission_index++; /* update global variable */
        USB.println(F("send ok"));
      }
      else 
      {
        USB.println(F("send error"));
      }
    }
    else
    {
      /**** if we are moving, abort transmission ****/
    
      /* close SD card */
      SD.OFF();
  
      /* turn ZigBee off, to save power */
      xbee868LP.OFF();
    
      /* return to other code execution */
      return;
    }
    
  }
  
  /**** transmission ended, close modules ****/
  /* close SD card */
  SD.OFF();
 
  /* turn ZigBee off, to save power */
  xbee868LP.OFF();
}

/*
 * 
 */
void setup_sd_card(void)
{
  /* Set SD ON, to log samples */
  SD.ON();

  /* Delete file if already exists, to discard old samples */
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

  /* create new file */
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

  USB.println("SD card was initialized");
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

  USB.println("Sensors were initialized");
}

/*
 * 
 */
void setup_gps_and_rtc(void)
{
  /* define status variable for GPS connection */
  bool status;
  
  /* Init SD pins */
  SD.ON();

  /* Turn GPS on */
  GPS.ON();

  /* load ephemeris if applicable */
  GPS.loadEphems();
  
  /////////////////////////////////////////    
  // wait for GPS signal for specific time
  ////////////////////////////////////////
  status = GPS.waitForSignal(TIMEOUT);
  
  //////////////////////////////////////////////////////////////////////// 
  // if GPS is connected then store/load ephemeris to enable hot start
  //////////////////////////////////////////////////////////////////////// 
  if( status == true )
  {        
      /* store ephemeris in "EPHEM.TXT" */
      GPS.saveEphems();

      /* set time in RTC from GPS time (GMT time) */
      GPS.setTimeFromGPS();
  }


  /* at this point, compensate the time, to match the Greek time */
  /* add 3 hours to the hour that was taken from the GPS */
  RTC.setTime(RTC.year, RTC.month, RTC.date, RTC.dow(RTC.year, RTC.month, RTC.day), RTC.hour + 3, RTC.minute, RTC.second);
  
  /* switch SD card off */
  SD.OFF();
  
  /* switch GPS off */
  GPS.OFF();

  USB.println("GPS was initialized");

  USB.println("RTC was initialized");
}

 /*
 * 
 */
bool check_the_date(void)
{
  bool day_flag_updated = false;
  int temp_hour = 0;
  
  /* see the day (eg. is it Monday, Tuesday etc.) */
  char* Date = strtok(RTC.getTime(),",");

  if(strcmp(Date, temp_day)) /* if the day hasn't changed, don't update the flag */
  {
    day_flag_updated = false;
  }
  else /* if a day has passed, update the flag */
  {
    temp_day = Date; /* update the temp day variable */
    day_flag_updated = true; /* update the flag, so the data can be sent */
  }

  temp_hour = (int)RTC.hour; /* check what time it is */

  /* update the daytime */
  if(temp_hour <= 6 || temp_hour >= 19)
  {
    daytime = "NIGHT"; /* between 19:00 and 6:00, it is night */
  }
  else
  {
    daytime = "DAY"; /* else, it is day */
  }

  /* if one day has passed, return TRUE */
  if(day_flag_updated)
  {
    return true;
  }
  else /* else return FALSE */
  {
    return false;
  }
}

/*
 * 
 */
void run_application(void)
{
    /* enable sensors board */
    Gases.ON();

    /* enable the CO sensor */
    COSensor.ON();


    #ifdef MODE2 /* if we work on full demo, start other sensors */
    
    CO2Sensor.ON();
    NO2Sensor.ON();
    
    #endif

    /* wait 90 seconds for the sensors to warm-up */
    ninety_seconds_delay();

    
    /**** READ SENSORS ****/
    Read_Sensors();

    #ifdef PDEBUG /* only print to serial if the debug mode is set */
    /**** PRINT OF THE RESULTS ****/
    Print_Sensors_Values();
    #endif

    /**** CREATE ASCII FRAME ****/
    Create_Ascii();

    /**** CHECK IF WE ARE STOPPED AND THERE IS A ZIGBEE STATION NEARBY ****/
    Send_Data();

    /* if a day has passed, send the logged data to Meshlium */
    if(check_the_date() == true)
    {
      /* power up SD card */
      SD.ON();
      
      /* power up the GPS */
      GPS.ON();

      /* load ephemeris */
      GPS.loadEphems();

      /* send the data to Meshlium */
      read_logged_data_and_send(transmission_index);

      /* close the GPS module */
      GPS.OFF();

      /* close SD card */
      SD.OFF();
    }
}

/*
 * 
 */
void Send_Data(void)
{
  /* speed variable will check if the vehicle is moving or not */
  int vehicle_speed = 0;

  /* power up SD card */
  SD.ON();
  
  /* power up GPS */
  GPS.ON();

  // load ephemeris previously stored in SD
  GPS.loadEphems();
  
  status = GPS.waitForSignal(TIMEOUT);

  if( status == true ) /* we have GPS signal */
  {
    /* check if the vehicle is not moving */
    vehicle_speed = (int)GPS.speed;

    if (vehicle_speed < 5) // vehicle has slowed down or not moving
    {
      /* now it is a good time to check if there is a Zigbee Meshlium nearby */
      read_logged_data_and_send(transmission_index);
    }
    else
    {
      /**** we are moving , abort transmission ****/

      /* close the GPS module */
      GPS.OFF();

      /* close SD card */
      SD.OFF();
  
      /* return to other code execution */
      return;
    }
  }

  /* close the GPS module */
  GPS.OFF();

  /* close SD card */
  SD.OFF();
}

 /*
  * 
  */
void ninety_seconds_delay(void)
{
  uint8_t i = 0; /* 90 seconds counter */

  for(i = 0; i < 90; i++)
  {
    delay(1000);
  }
}

/*
 * 
 */
 void see_battery_status(void)
 {
    bool chargeState;
    uint16_t chargeCurrent;

    /* get charging state and current */
    chargeState = PWR.getChargingState();
    chargeCurrent = PWR.getBatteryCurrent();

    if (chargeState == true)
    {
      USB.println(F("Battery is charging"));
    }
    else
    {
      USB.println(F("Battery is not charging"));
    }

    /* Show the battery charging current (only from solar panel) */
    USB.print(F("Battery charging current (only from solar panel): "));
    USB.print(chargeCurrent, DEC);
    USB.println(F(" mA"));

    USB.println();
 }
