/*******************************************************************
    example woth board no.1.

    This board has the CO and the Temp-Hum-Press sensors on it.

    Debug Mode -> prints info to serial as well


    Author: Nikolaos Monios , moniosni@gmail.com

    MSc. Thesis for the University of West Attika
*******************************************************************/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBee868LP.h>
#include <WaspGPS.h>
#include "coefficients.h"

/*#define PDEBUG 1*/  /* enable serial print for debug reasons */
/*#define MODE2  1*/    /* mode 2, when enabled, will read more sensors */

void setup()
{
  #ifdef PDEBUG
  USB.ON();   /* configure the USB port */
  USB.println(F("Board is starting..."));
  #endif

  /* Set the Waspmote ID */
  frame.setID(node_ID);

  /* setup the sensors that will be used in this application */
  Setup_App_Sensors();

  /* setup the SD card */
  Setup_SD_Card();

  /* setup the GPS and RTC modules */
  Setup_GPS_And_RTC();
  
  #ifdef PDEBUG
  USB.println(F("starting application now"));
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
        Run_Application(); /* main routine */

        /* check if battery is charging from solar panel, in debug mode */
        #ifdef PDEBUG
        See_Battery_Status();
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
        Run_Application(); /* main routine */
      }
    }
  }
  
}

/******************************* FUNCTIONS *******************************/

/*
 * Function: Read_Sensors
 * ----------------------
 *    Reads all the sensors of the application,
 *    and stores their values to global variables
 *    (CO2, NO2, Temperature etc)
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Read_Sensors(void)
{
  /* Start the GPS*/
  GPS.ON();

  /* load ephemeris previously stored in SD */
  //GPS.loadEphems();
  
  status = GPS.waitForSignal(TIMEOUT); /* wait some time for the GPS to connect */

  if( status == true ) /* if connection was established, read the sensors */
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
}

/*
 * Function: Print_Sensors_Values
 * ------------------------------
 *    Prints the values of the sensors
 *    to Serial Monitor, in Debug mode
 *    
 *    Input:  none
 *    
 *    Output: none
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
 * Function: Create_Ascii
 * ----------------------
 *    Add the values of the sensors
 *    to the frame, and write the frame
 *    to the SD card
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Create_Ascii(void)
{
  /* power up SD card */
  SD.ON();
  
  /* Create new frame (ASCII) */
  frame.createFrame(ASCII); /* frame.createFrame(ASCII, node_ID) */

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

  /*  get the actual time from thr RTC */
  RTC.getTime();

  /* add the timestamp to the samples */
  frame.addTimestamp();

  /* print the frame for debug reasons */
  #ifdef PDEBUG
  frame.showFrame();
  #endif

  /********* store these samples to the dedicated file ********/
  /************************************************************/
  /* save the frame to the SD card now */
  /* init the buffer that will hold the frame info */
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
      USB.println(F("Frame WITHOUT GPS appended to file"));
      #endif
  }
  else 
  {
      #ifdef PDEBUG
      USB.println(F("Append of data WITHOUT GPS failed"));
      #endif
  }

  /********* store the GPS coordinates to the dedicated file now ********/
  /**********************************************************************/

  /* first, create a new frame that will hold the GPS coordinates */
  frame.createFrame(ASCII);

  /* now add the coordinates */
  frame.addSensor(SENSOR_GPS, 
                          GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator),
                          GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator) );

  /* now add the same timestamp */
  frame.addTimestamp();

  /* print the frame for debug reasons */
  #ifdef PDEBUG
  frame.showFrame();
  #endif

  /* save the frame to the SD card  now */
  /* init the buffer that will hold the frame info */
  memset(toWrite, 0x00, sizeof(toWrite) );

  /* Conversion from Binary to ASCII */
  Utils.hex2str( frame.buffer, toWrite, frame.length);
  
  /* some debug help here */
  USB.print(F("Frame to be stored:"));
  USB.println(toWrite);

  /* now append data to file */
  sd_answer = SD.appendln(filename_gps, toWrite);
  
  if( sd_answer == 1 )
  {
      #ifdef PDEBUG
      USB.println(F("Frame WITH GPS appended to file"));
      #endif
  }
  else 
  {
      #ifdef PDEBUG
      USB.println(F("Append of data WITH GPS failed"));
      #endif
  }

  /* close GPS - it has been on since the Read_Sensors() was called */
  GPS.OFF();
  
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
 * Function: Read_GPS_Coordinates_And_Send
 * ---------------------------------------
 *    Parses the SD card for the GPS coordinates
 *    that need to be sent to the Meshlium,
 *    and sends them using XBee
 *    
 *    Input:  index
 *            Indicates the line of the file
 *            containing the coordinates
 *    
 *    Output: none
 */
void Read_GPS_Coordinates_And_Send(int index)
{
  /* speed variable will check if the vehicle is moving or not */
  int vehicle_speed = 0;
  
  /* init XBee */
  xbee868LP.ON();

  /* open SD card */
  SD.ON();
  
  /* get number of lines in file */
  numLines = SD.numln(filename_gps);

  /* get specified lines from file
     get only the last file line*/
  startLine = index;
  endLine = numLines;

  /* iterate to get the File lines specified */
  for( int i = startLine; i < endLine ; i++ )
  {  
    /* Get 'i' line -> SD.buffer */
    SD.catln( filename_gps, i, 1); 
    
    /* initialize frameSD */
    memset(frameSD, 0x00, sizeof(frameSD) ); 
    
    /* conversion from ASCII to Binary */
    lengthSD = Utils.str2hex(SD.buffer, frameSD );


    /* not actually needed here -- just for debug */
    /* Conversion from ASCII to Binary */
    #ifdef PDEBUG
    USB.print(F("Get previously stored GPS frame:"));
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
        transmission_index_gps++; /* update gps data index */
        USB.println(F("send GPS ok"));
      }
      else 
      {
        USB.println(F("send GPS error"));
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
 * Function: Read_Logged_Data_And_Send
 * -----------------------------------
 *    Parses the SD card for the values of
 *    the sensors that need to be sent to 
 *    the Meshlium, and sends them using XBee
 *    
 *    Input:  index
 *            Indicates the line of the file
 *            containing the coordinates
 *    
 *    Output: none
 */
void Read_Logged_Data_And_Send(int index)
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
		transmission_index_data++; /* update sensor data index */
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
 * Function: Setup_SD_Card
 * -----------------------
 *    Deletes old files and creates
 *    new files for logging the data
 *    from the sensors and the GPS
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Setup_SD_Card(void)
{
  /* Set SD ON, to log samples */
  SD.ON();

  /* DELETE FILE WITH SAMPLES */
  /* Delete file if already exists, to discard old samples */
  sd_answer = SD.del(filename);

  if(sd_answer) 
  {
    #ifdef PDEBUG
    USB.println(F("file containing samples deleted"));
    #endif
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("file containing samples NOT deleted"));
    #endif
  }

  /* create new file */
  sd_answer = SD.create(filename);

  if(sd_answer)
  { 
    #ifdef PDEBUG
    USB.println(F("file containing samples created"));
    #endif
  }
  else
  {
    #ifdef PDEBUG
    USB.println(F("file containing samples NOT created"));
    #endif
  }
  /***********************************************************/

  /* DELETE FILE WITH GPS */
  /* Delete file if already exists, to discard old samples */
  sd_answer = SD.del(filename_gps);

  if(sd_answer) 
  {
    #ifdef PDEBUG
    USB.println(F("file containing GPS coordinates deleted"));
    #endif
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("file containing GPS coordinates NOT deleted"));
    #endif
  }

  /* create new file */
  sd_answer = SD.create(filename_gps);

  if(sd_answer)
  { 
    #ifdef PDEBUG
    USB.println(F("file containing GPS coordinates created"));
    #endif
  }
  else
  {
    #ifdef PDEBUG
    USB.println(F("file containing GPS coordinates NOT created"));
    #endif
  }
  
  USB.println(F("SD card was initialized"));
}

/*
 * Function: Setup_App_Sensors
 * ---------------------------
 *    Calculates the logarithmic functions
 *    that are going to be used by non-linear
 *    sensors
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Setup_App_Sensors(void)
{
  /* Calculate the slope and the intersection of the logarithmic functions */
  COSensor.setCalibrationPoints(COres, COconcentrations, numPoints);           /* for CO  sensor */

  
  #ifdef MODE2 /* if we work on full demo, load other sensors */
  
  CO2Sensor.setCalibrationPoints(CO2voltages, CO2concentrations, numPoints);   /* for CO2 sensor */
  
  NO2Sensor.setCalibrationPoints(NO2res, NO2concentrations, numPoints);        /* for NO2 sensor */
  
  #endif

  USB.println(F("Sensors were initialized"));
}

/*
 * Function: Setup_GPS_And_RTC
 * ---------------------------
 *    Starts the GPS and stores the
 *    Ephemeris information to the SD card
 *    so it can start faster during the
 *    application. Sets the real time and 
 *    date to the RTC
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Setup_GPS_And_RTC(void)
{
  /* define status variable for GPS connection */
  bool status;
  
  /* Init SD pins */
  SD.ON();

  /* Turn GPS on */
  GPS.ON();
  
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
      //GPS.saveEphems();

      #ifdef PDEBUG
      USB.println(F("Ephemeries were saved in a file"));
      #endif

      /* set time in RTC from GPS time (GMT time) */
      GPS.setTimeFromGPS();

      /* at this point, compensate the time, to match the Greek time */
      /* add 3 hours to the hour that was taken from the GPS */
      RTC.setTime(RTC.year, RTC.month, RTC.date, RTC.dow(RTC.year, RTC.month, RTC.day), RTC.hour + 3, RTC.minute, RTC.second);
  
      /* switch SD card off */
      SD.OFF();
  
      /* switch GPS off */
      GPS.OFF();

      USB.println(F("GPS was initialized"));

      USB.println(F("RTC was initialized"));

      Check_The_Date(); /* update the date, we do not want to have the initialized values - maybe the app will start during the night */
  }

}

 /*
 * Function: Check_The_Date
 * ------------------------
 *    Checks if a day has passed since
 *    we started the application. Checks
 *    if it is day or night outside
 *    
 *    Input:  none
 *    
 *    Output: TRUE    if a day has passed
 *            FALSE   if no day has passed
 */
bool Check_The_Date(void)
{
  bool day_flag_updated = false;
  int temp_hour = 0;
  
  /* see the day (eg. is it Monday, Tuesday etc.) */
  char* Date = strtok(RTC.getTime(),",");

  if(strcmp(Date, temp_day) == 0) /* if the day hasn't changed, don't update the flag */
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
  if(temp_hour >= morning_limit && temp_hour <= night_limit)
  {
    daytime = "DAY"; /* between 19:00 and 6:00, it is night */
  }
  else
  {
    daytime = "NIGHT"; /* else, it is day */
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
 * Function: Run_Application
 * -------------------------
 *    Starts the sensor board. Starts the sensors
 *    and wait 90 seconds for them to warm up.
 *    Reads samples from sensors, stores them
 *    to frames and send them via XBee if
 *    necessary.
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Run_Application(void)
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
    Ninety_Seconds_Delay();

    
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
    if(Check_The_Date() == true)
    {
      /* power up SD card */
      SD.ON();
      
      /* power up the GPS */
      GPS.ON();

      /* load ephemeris */
      /* GPS.loadEphems(); */

      /* send the samples to Meshlium */
      Read_Logged_Data_And_Send(transmission_index_data);

      /* send the coordinates of the sent sample to Meshlium */
      Read_GPS_Coordinates_And_Send(transmission_index_gps);

      /* close the GPS module */
      GPS.OFF();

      /* close SD card */
      SD.OFF();
    }
}

/*
 * Function: Send_Data
 * -------------------
 *    If the bus is not moving,
 *    send frames via XBee
 *    
 *    Input:  none
 *    
 *    Output: none
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
  /* GPS.loadEphems(); */
  
  status = GPS.waitForSignal(TIMEOUT);

  if( status == true ) /* we have GPS signal */
  {
    /* check if the vehicle is not moving */
    vehicle_speed = (int)GPS.speed;

    if (vehicle_speed < 5) // vehicle has slowed down or not moving
    {
      /* now it is a good time to check if there is a Zigbee Meshlium nearby and send it samples */
      Read_Logged_Data_And_Send(transmission_index_data);

      /* send the coordinates of the sent sample to Meshlium */
      Read_GPS_Coordinates_And_Send(transmission_index_gps);
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
 * Function: Ninety_Seconds_Delay
 * ------------------------------
 *    waits 90 seconds for the gases
 *    sensors to warm up
 *    
 *    Input:  none
 *    
 *    Output: none
 */
void Ninety_Seconds_Delay(void)
{
  uint8_t i = 0; /* 90 seconds counter */

  for(i = 0; i < 90; i++)
  {
    delay(1000);
  }
}

/*
 * Function: See_Battery_Status
 * ----------------------------
 *    If we are in debug mode, print
 *    to the serial monitor the status
 *    of the battery
 *    
 *    Input:  none
 *    
 *    Output: none
 */
 void See_Battery_Status(void)
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
