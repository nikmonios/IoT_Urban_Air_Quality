/*
    example woth board no.1.

    This board has the NO2 and the Luminosity sensors on it.

    Read sensors and print them to Serial Screen
*/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBeeZB.h>
#include "coefficients.h"

#define PDEBUG 1   /* enable serial print for debug reasons */
/*#define MODE2  1*/    /* mode 2, when enabled, will read all sensors */

/* variables for data logging */
// define file name: MUST be 8.3 SHORT FILE NAME
char filename[]="FILE1.TXT";
// buffer to write into Sd File
char toWrite[256];
// define variables to read stored frames 
uint8_t frameSD[MAX_FRAME+1];
uint16_t lengthSD;
int32_t numLines;
// variables to define the file lines to be read
int startLine;
int endLine;
// define variable
uint8_t sd_answer;


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

  /* init XBee */
  xbeeZB.ON();
  
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

  if (timer_unit == 15) /* have we got 15 8_Sec interrupts (120 secs)? */
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
    #ifdef PDEBUG
    Create_Ascii();
    #endif
  }
}

/**** FUNCTIONS ****/

void Read_Sensors()
{
  /***********************************************/
  /* READ TEMPERATURE, HUMIDITY, AND PRESSURE    */
  #ifdef MODE2
  delay(4000);
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


void Print_Sensors_Values()
{
  #ifdef MODE2
  USB.print(F(" Temperature: "));
  USB.flush();
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

void Create_Ascii()
{
  /* first see if we have achieved communication with Meshlium */
  error = xbeeZB.setRTCfromMeshlium(MESHLIUM_ADDRESS);

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

  if ( error == 0) /* if we have the time from the Meshlium */
  {
    /* add timestamp from RTC */
    RTC.getTime();
    delay(10);
    frame.addTimestamp();
  }
  
  /* Show the frame */
  #ifdef PDEBUG
  frame.showFrame();
  #endif

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
  /* temperature = 0;
  humidity = 0;
  pressure = 0;*/
  COPPM = 0;
  #ifdef MODE2 /* if we work on full demo, clear other sensors */
  CO2PPM = 0;
  NO2PPM = 0;
  #endif
}

void read_log_data()
{
  // get number of lines in file
  numLines = SD.numln(filename);

  // get specified lines from file
  // get only the last file line
  startLine = numLines-1; 
  endLine = numLines;

  // iterate to get the File lines specified
  for( int i=startLine; i<endLine ; i++ )
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
    
    for(int j=0; j < lengthSD; j++)
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
  /* first check XBee's network parameters */
  checkNetworkParams();

  /* then, send the frame */
  error = xbeeZB.send( MESHLIUM_ADDRESS, frameSD, lengthSD );

  // check TX flag
  if( error == 0 )
  {
    #ifdef PDEBUG
    USB.println(F("send ok"));
    #endif
    
    // blink green LED
    Utils.blinkGreenLED();
  }
  else 
  {
    #ifdef PDEBUG
    USB.println(F("send error"));
    #endif
    
    // blink red LED
    Utils.blinkRedLED();
  }
}

void checkNetworkParams()
{
  // 1. get operating 64-b PAN ID
  xbeeZB.getOperating64PAN();

  // 2. wait for association indication
  xbeeZB.getAssociationIndication();
 
  while( xbeeZB.associationIndication != 0 )
  { 
    delay(2000);
    
    // get operating 64-b PAN ID
    xbeeZB.getOperating64PAN();

    #ifdef PDEBUG
    USB.print(F("operating 64-b PAN ID: "));
    USB.printHex(xbeeZB.operating64PAN[0]);
    USB.printHex(xbeeZB.operating64PAN[1]);
    USB.printHex(xbeeZB.operating64PAN[2]);
    USB.printHex(xbeeZB.operating64PAN[3]);
    USB.printHex(xbeeZB.operating64PAN[4]);
    USB.printHex(xbeeZB.operating64PAN[5]);
    USB.printHex(xbeeZB.operating64PAN[6]);
    USB.printHex(xbeeZB.operating64PAN[7]);
    USB.println();
    #endif  
    
    xbeeZB.getAssociationIndication();
  }

  #ifdef PDEBUG
  USB.println(F("\nJoined a network!"));
  #endif

  // 3. get network parameters 
  xbeeZB.getOperating16PAN();
  xbeeZB.getOperating64PAN();
  xbeeZB.getChannel();

  #ifdef PDEBUG
  USB.print(F("operating 16-b PAN ID: "));
  USB.printHex(xbeeZB.operating16PAN[0]);
  USB.printHex(xbeeZB.operating16PAN[1]);
  USB.println();
  #endif

  #ifdef PDEBUG
  USB.print(F("operating 64-b PAN ID: "));
  USB.printHex(xbeeZB.operating64PAN[0]);
  USB.printHex(xbeeZB.operating64PAN[1]);
  USB.printHex(xbeeZB.operating64PAN[2]);
  USB.printHex(xbeeZB.operating64PAN[3]);
  USB.printHex(xbeeZB.operating64PAN[4]);
  USB.printHex(xbeeZB.operating64PAN[5]);
  USB.printHex(xbeeZB.operating64PAN[6]);
  USB.printHex(xbeeZB.operating64PAN[7]);
  USB.println();
  #endif

  #ifdef PDEBUG
  USB.print(F("channel: "));
  USB.printHex(xbeeZB.channel);
  USB.println();
  #endif

}
