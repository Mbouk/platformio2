
#include "aFish2.h"
#include "esense_board_api.h"
#include "Wire.h"

AFish2 af;
int  V = 0;
int W = 0;
int es_status = 0;

void setup()
{
  // init with id number and name
  af.begin(0, "aFish_000");

  //                        Serial.end();
  //    Serial.begin(115200);

}

void loop()
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // just wait
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  delay(100);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // internal led
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // while(1)
  // {
  //     pinMode(5, OUTPUT);
  //     digitalWrite(5, HIGH);
  //     delay(500);
  //     digitalWrite(5, LOW);
  //     delay(500);
  // }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // led strip
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // af.ledstrip.setFront(Ledstrip::Hsvb(0, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setFront(Ledstrip::Hsvb(85, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setFront(Ledstrip::Hsvb(170, 255, 255, 10));
  // delay(1000);

  // af.ledstrip.setLeft(Ledstrip::Hsvb(0, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setLeft(Ledstrip::Hsvb(85, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setLeft(Ledstrip::Hsvb(170, 255, 255, 10));
  // delay(1000);

  // af.ledstrip.setRight(Ledstrip::Hsvb(0, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setRight(Ledstrip::Hsvb(85, 255, 255, 10));
  // delay(1000);
  // af.ledstrip.setRight(Ledstrip::Hsvb(170, 255, 255, 10));
  // delay(1000);

  // af.ledstrip.setAll(Ledstrip::Rgbb(0, 255, 0, 10));
  // delay(1000);
  // af.ledstrip.setAll(Ledstrip::Rgbb(0, 0, 255, 10));
  // delay(1000);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Propulsion + steering
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //     af.propulsion.set(2048);
  //     delay(2000);
  //     af.propulsion.set(0);
  //     delay(1000);
  //     af.propulsion.set(-2048);
  //     delay(2000);
  //     af.propulsion.set(0);
  //     delay(3000);
  //
  //     af.steering.set(2048);
  //     delay(2000);
  //     af.steering.set(0);
  //     delay(1000);
  //     af.steering.set(-2048);
  //     delay(2000);
  //     af.steering.set(0);
  //     delay(3000);
  //
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Buoyancy
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // use a lambda function to handle bluetooth messages
  //     std::function<void(char*)> btHandler = [](char* buffer)
  //         {
  //             af.println("Got message");
  //
  //             char delims[] = {' ', '\n', '\r', NULL};
  //             char* tok = strtok (buffer, delims);
  //             if (tok != NULL)
  //             {
  //                 if (strcmp(tok, "up") == 0)
  //                 {
  //                     af.println("Up");
  //                     af.buoyancy.set(2048);
  //                 }
  //                 if (strcmp(tok, "stop") == 0)
  //                 {
  //                     af.println("Stop");
  //                     af.buoyancy.set(0);
  //                 }
  //                if (strcmp(tok, "down") == 0)
  //                 {
  //                     af.println("Down");
  //                     af.buoyancy.set(-2048);
  //                 }
  //                 if (strcmp(tok, "read") == 0)
  //                 {
  //                     af.println("Get position");
  //                     int v = af.buoyancy.getPosition();
  //                     af.println("Position=" + String(v));
  //                     af.println("Limit up=" + String((int)af.buoyancy.z));
  //                     af.println("Limit down=" + String((int)af.buoyancy.encoderLowerLimit));
  //                     af.println("Limited up=" + String((int)af.buoyancy.reachedUpperLimit()));
  //                     af.println("Limited down=" + String((int)af.buoyancy.reachedLowerLimit()));
  //
  //                 }
  //                 if (strcmp(tok, "reset") == 0)
  //                 {
  //                     af.println("Reset");
  //                     tok = strtok(NULL, delims);
  //                     int val = atoi(tok);
  //
  //                     int v = af.buoyancy.getPosition();
  //                     af.buoyancy.resetPosition(val);
  //                     af.println("Reset position=" + String(v));
  //                 }
  //                 if (strcmp(tok, "limitup") == 0)
  //                 {
  //                     tok = strtok(NULL, delims);
  //                     int val = atoi(tok);
  //
  //                     af.println("Set limit up" + String(val));
  //                     af.buoyancy.setUpperLimit(val);
  //                 }
  //                 if (strcmp(tok, "limitdown") == 0)
  //                 {
  //                     tok = strtok(NULL, delims);
  //                     int val = atoi(tok);
  //
  //                     af.println("Set limit down" + String(val));
  //                     af.buoyancy.setLowerLimit(val);
  //                 }
  //             }
  //         };
  //
  //     // set up function as callback which will handle all the work
  //     af.setBluetoothTerminalCallback(btHandler);
  //
  //     while(1)
  //     {
  //         delay(1000);
  //     }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // IMU
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //af.imu.readSensor();

  // af.print(af.imu.getAccelX_mss(),6);
  // af.print("\t");
  // af.print(af.imu.getAccelY_mss(),6);
  // af.print("\t");
  // af.print(af.imu.getAccelZ_mss(),6);
  // af.print("\t");
  // af.print(af.imu.getGyroX_rads(),6);
  // af.print("\t");
  // af.print(af.imu.getGyroY_rads(),6);
  // af.print("\t");
  // af.print(af.imu.getGyroZ_rads(),6);
  // af.print("\t");
  // af.print(af.imu.getMagX_uT(),6);
  // af.print("\t");
  // af.print(af.imu.getMagY_uT(),6);
  // af.print("\t");
  // af.print(af.imu.getMagZ_uT(),6);
  // af.print("\t");
  // af.println(af.imu.getTemperature_C(),6);
  // delay(500);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // GPS
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // af.gps.read();

  // af.print("Location: ");
  // if (af.gps.location.isValid())
  // {
  //     af.print(af.gps.location.lat(), 6);
  //     af.print(",");
  //     af.print(af.gps.location.lng(), 6);
  // }
  // else
  // {
  //     af.print("INVALID");
  // }

  // af.print("  Date/Time: ");
  // if (af.gps.date.isValid())
  // {
  //     af.print(af.gps.date.month());
  //     af.print("/");
  //     af.print(af.gps.date.day());
  //     af.print("/");
  //     af.print(af.gps.date.year());
  // }
  // else
  // {
  //     af.print("INVALID");
  // }

  // af.print(" ");
  // if (af.gps.time.isValid())
  // {
  //     if (af.gps.time.hour() < 10) af.print("0");
  //     af.print(af.gps.time.hour());
  //     af.print(":");
  //     if (af.gps.time.minute() < 10) af.print("0");
  //     af.print(af.gps.time.minute());
  //     af.print(":");
  //     if (af.gps.time.second() < 10) af.print("0");
  //     af.print(af.gps.time.second());
  //     af.print(".");
  //     if (af.gps.time.centisecond() < 10) af.print("0");
  //     af.print(af.gps.time.centisecond());
  // }
  // else
  // {
  //     af.print("INVALID");
  // }

  // af.println();
  // delay(1000);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Pressure / temperature
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // af.pt.read();

  // af.print("Pressure: ");
  // af.print(af.pt.pressure());
  // af.println(" mbar");

  // af.print("Depth: ");
  // af.print(af.pt.depth());
  // af.println(" m");

  // af.print("Altitude: ");
  // af.print(af.pt.altitude());
  // af.println(" m above mean sea level");

  // af.print("Temperature: ");
  // af.print(af.pt.temperature());
  // af.println(" deg C");

  // delay(1000);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Nanomodem
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // af.modem.ping(124);
  // delay(1000);
  // af.modem.send(99, "hello");

  // long t = millis();
  // af.println("Listening for incoming acoustic message...");
  // char msg[10];
  // int len = 0;
  // while (millis() - t < 10000)
  // {
  //     if (af.modem.available())
  //     {
  //         len = af.modem.receive(msg);
  //         break;
  //     }
  // }
  // if (len > 0)
  // {
  //     af.print("Received message ");
  //     af.println(msg);
  // }
  // else
  // {
  //     af.print("No message received");
  // }

  // delay(1000);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Sonar ranger
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  //  byte error, address;
  //  int nDevices;
  //
  //  af.println("Scanning...");
  //
  //  nDevices = 0;
  //  for(address = 1; address < 127; address++ )
  //  {
  //    // The i2c_scanner uses the return value of
  //    // the Write.endTransmisstion to see if
  //    // a device did acknowledge to the address.
  //    Wire.beginTransmission(address);
  //    error = Wire.endTransmission();
  //
  //    if (error == 0)
  //    {
  //      af.print("I2C device found at address 0x");
  //      if (address<16)
  //        af.print("0");
  //      af.print(address,HEX);
  //      af.println("  !");
  //
  //      nDevices++;
  //    }
  //    else if (error==4)
  //    {
  //      af.print("Unknown error at address 0x");
  //      if (address<16)
  //        af.print("0");
  //      af.println(address,HEX);
  //    }
  //  }
  //  if (nDevices == 0)
  //    af.println("No I2C devices found\n");
  //  else
  //    af.println("done\n");
  //
  // // delay(5000);           // wait 5 seconds for next scan
  //
  //  delay(500);
  //
  //    af.sonarFront.sample();
  //    delay(300);
  //    int v = af.sonarFront.read();
  //    af.println("Sonar read            : " + String(v));
  //
  //    // this one takes 100ms in total
  //    v = af.sonarFront.sampleAndRead();
  //    af.println("Sonar sample and read : " + String(v));
  //    delay(300);
  //
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Motors
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //     af.println("Forward propulsion");
  //     af.propulsion.set(50);
  //     delay(2000);
  //     af.println("Backward propulsion");
  //     af.propulsion.set(-50);
  //     delay(2000);
  //
  //     af.println("Left steering");
  //     af.steering.set(50);
  //     delay(2000);
  //     af.println("Right steering");
  //     af.steering.set(-50);
  //     delay(2000);
  //
  //     af.println("Buoyancy +");
  //     af.buoyancy.set(50);
  //     delay(2000);
  //     af.println("Buoyancy -");
  //     af.buoyancy.set(-50);
  //     delay(2000);
  //
  //     delay(5000);


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // E sense
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // std::function<void(char*)> btHandler = [](char* buffer)
  //         {
  //             af.println("Got message");
  //
  //             char delims[] = {' ', '\n', '\r', NULL};
  //             char* tok = strtok (buffer, delims);
  //             if (tok != NULL)
  //             {
  //                 if (strcmp(tok, "eson") == 0)
  //                 {
  //                     af.println("enable Esense on");
  //                     af.enableEsense(true);
  //                     delay(200);
  //                     af.println("Esense Start Emission");
  //                     Esense_Board_Start_Emission();
  //                     delay(10000);//time to put in water
  //                     af.println("Esense Autoset Amplitude");
  //                     Esense_Board_Autoset_Amplitude();
  //                     delay(200);//time to put in water
  //                     Esense_Board_Gain_Amplitude(&V);
  //                     af.println("Esense amplitude");
  //                     af.print(V, DEC);
  //                     es_status = 0;
  //                 }
  //                 if (strcmp(tok, "esoff") == 0)
  //                 {
  //                     af.println("enable Esense off");
  //                     af.enableEsense(false);
  //                      delay(200);
  //                     af.println("Esense Start Emission");
  //                     Esense_Board_Start_Emission();
  //                     delay(10000);//time to put in water
  //                     af.println("Esense Autoset Amplitude");
  //                     Esense_Board_Autoset_Amplitude();
  //                     delay(200);//time to put in water
  //                     Esense_Board_Gain_Amplitude(&V);
  //                     af.println("Esense amplitude");
  //                     af.print(V, DEC);
  //                     es_status = 0;
  //                 }
  //                 if (strcmp(tok, "stop") == 0)
  //                 {
  //                     af.propulsion.set(0);
  //                     af.steering.set(0);
  //                     af.enableEsense(false);
  //                     es_status = 0;
  //                 }
  //                 if (strcmp(tok, "esavoid") == 0)
  //                 {
  //                    es_status = 1;
  //                 }
  //             }
  //         };
  //
  //     // set up function as callback which will handle all the work
  //     af.setBluetoothTerminalCallback(btHandler);

  //     while(1)
  //     {
  //         delay(1000);
  //
  //     }
  // if (es_status == 1)
  //         {
  //                     af.println("Esense Do Measurement \n");
  //                     Esense_Board_Do_Measurement();
  //                     af.println("Esense V");
  //                     Esense_board_get_V(&V);
  //                     af.print(V, DEC);
  //                     af.printf("\n");
  //                     if (V>200)
  //                     {
  //                         V = 200;
  //                     }
  //
  //                      if ((V<50) || (V>150))
  //                      {
  //                         af.printf("Go forward \n");
  //                         //af.propulsion.set(2048);
  //                      }
  //                      else if ((V>70) && (V<130))
  //                      {
  //                          //motorV->goFish(2,-0.30);//pola moteur inversée
  //                          //motorV->goFish(3,-0.40);//pola moteur inversée
  //                           af.printf("Go backward \n");
  //                           //af.propulsion.set(-2048);
  //                      }
  //
  //                      Esense_Board_get_W(&W);
  //                      af.printf("yawing ");
  //                      af.print(W, DEC);
  //                      af.printf("\n");
  //                      if (W>200)
  //                      {
  //                          W = 200;
  //                      }
  //
  //                      //motorV->goFish(1,0.99*(W-100)/100);
  //                      if (W<90)
  //                      {
  //                        af.printf("Go left \n");
  //                        //af.steering.set(-20*(100-W));
  //                      }
  //                      else if (W>110)
  //                      {
  //                          af.printf("Go right \n");
  //                          //af.steering.set(20*(W-100));
  //                      }
  //                      else
  //                      {
  //                        af.printf("Go forrward \n");
  //                        //af.steering.set(0);
  //                      }
  //          }
  //          else
  //          {
  //            //af.propulsion.set(-2048);
  //           //af.steering.set(0);
  //           }

  byte error, address;
  int nDevices;
  af.enableEsense(true);
  af.enableGreenlight(true);
  af.println("Scanning.es=1..");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      af.print("I2C device found at address 0x");
      if (address < 16)
        af.print("0");
      af.print(address, HEX);
      af.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      af.print("Unknown error at address 0x");
      if (address < 16)
        af.print("0");
      af.println(address, HEX);
    }
  }
  if (nDevices == 0)
    af.println("No I2C devices found\n");
  else
    af.println("done\n");

  delay(5000);           // wait 5 seconds for next scan

  af.enableEsense(false);
  af.println("Scanning.es=0..");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      af.print("I2C device found at address 0x");
      if (address < 16)
        af.print("0");
      af.print(address, HEX);
      af.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      af.print("Unknown error at address 0x");
      if (address < 16)
        af.print("0");
      af.println(address, HEX);
    }
  }
  if (nDevices == 0)
    af.println("No I2C devices found\n");
  else
    af.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
