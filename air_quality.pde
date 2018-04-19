/*
    example woth board no.1.

    This board has the NO2, CO2, CO, and Temperature-Pressure-Humidity sensors on it.

    Read sensors and print them to Serial Screen Periodically
*/

#include <WaspSensorGas_v30.h>
#include <WaspFrame.h>
#include <WaspXBeeZB.h>
#include "includes.h"

void setup()
{
  // Configure the USB port
  USB.ON();
  USB.println(F("Board is starting..."));

  // Calculate the slope and the intersection of the logarithmic function
  CO2Sensor.setCalibrationPoints(CO2voltages, CO2concentrations, numPoints);
  // Calculate the slope and the intersection of the logarithmic function
  COSensor.setCalibrationPoints(COres, COconcentrations, numPoints);
  // Calculate the slope and the intersection of the logarithmic function
  NO2Sensor.setCalibrationPoints(NO2res, NO2concentrations, numPoints);

  // Set the Waspmote ID
  frame.setID(node_ID);

  RTC.ON(); //

  error = xbeeZB.setRTCfromMeshlium(MESHLIUM_ADDRESS);
  if ( error == 0) //if we have the time from the Meshlium
  {
    //add timestamp from RTC
    RTC.getTime();
    delay(10);
    USB.println("took real time and date from Meshlium");
  }
  
  USB.println("starting application now");
}


void loop()
{
  PWR.sleep(WTD_4S, ALL_OFF); //sleep for 4 seconds, kill all sensors and boards
  USB.ON();
  if( intFlag & WTD_INT )
  
  {
    USB.println(F("---------------------"));
    USB.println(F("WTD INT captured"));
    USB.println(F("---------------------"));
    intFlag &= ~(WTD_INT); //when you wake up, clean the flag
  }

  timer_unit += 1; //update the timer unit every time you wake up

  if (timer_unit == 3) // have we got 3 4_Sec interrupts (12 secs)?
  {
    timer_unit = 0; //reset the timer unit

    // init XBee
    xbeeZB.ON();

    //and take measurements
    Gases.ON();
    COSensor.ON();
    CO2Sensor.ON();
    NO2Sensor.ON();
    delay(100);
  
    Read_Sensors();       /**** READ SENSORS ****/
    Print_Sensors_Values(); /**** PRINT OF THE RESULTS ****/
    Create_Ascii();        /**** CREATE ASCII FRAME ****/
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
  COPPM = COSensor.readConcentration(); // PPM value of CO
  /***********************************************/
  
  /*                READ CO2                      */
  CO2PPM = CO2Sensor.readConcentration();
  /**********************************************/
  
  /*                READ NO2                      */
  NO2PPM = NO2Sensor.readConcentration();
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

  USB.print(F(" CO2 concentration estimated: "));
  USB.print(CO2PPM);
  USB.println(F(" ppm"));

  USB.print(F(" CO concentration Estimated: "));
  USB.print(COPPM);
  USB.println(F(" ppm"));

  USB.print(F(" NO2 concentration Estimated: "));
  USB.print(NO2PPM);
  USB.println(F(" ppm"));
}

void Create_Ascii()
{
  //first see if we have achieved communication with Meshlium
  error = xbeeZB.setRTCfromMeshlium(MESHLIUM_ADDRESS);

  // Create new frame (ASCII)
  frame.createFrame(ASCII, node_ID);
  // Add temperature
  frame.addSensor(SENSOR_GASES_TC, temperature);
  // Add humidity
  frame.addSensor(SENSOR_GASES_HUM, humidity);
  // Add pressure
  frame.addSensor(SENSOR_GASES_PRES, pressure);
  // Add CO2 PPM value
  frame.addSensor(SENSOR_GASES_CO2, CO2PPM);
  // Add CO PPM value
  frame.addSensor(SENSOR_GASES_CO, COPPM);
  // Add CO PPM value
  frame.addSensor(SENSOR_GASES_NO2, NO2PPM);

  if ( error == 0) //if we have the time from the Meshlium
  {
    //add timestamp from RTC
    RTC.getTime();
    delay(10);
    frame.addTimestamp();
  }
  // Show the frame
  frame.showFrame();

  /* reset values */
  temperature = 0;
  humidity = 0;
  pressure = 0;
  CO2PPM = 0;
  COPPM = 0;
  NO2PPM = 0;
}

