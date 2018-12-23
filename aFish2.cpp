#include "aFish2.h"

#include <Update.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_system.h>

// ----------------------------------------------------------------------------

void AFish2::begin(int id, char* name)
{
     this->id = id;
     this->name = name;

     // init i2c
     Wire.begin();
     Wire.setClock(400000);        

     // init bluetooth serial by default at startup, user can disable it later
     SerialBT.begin(name);
     beginBluetoothTerminal();        
    
     // init gpio/pwm expander
     pwm.begin();
     pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
     delay(100);
    
     // init ledstrip
     ledstrip.begin();
    
     // disable modules by default
     enableGreenlight(false);
     enableEsense(false);
    
     // init IMU
     int status = imu.begin();
     if (status < 0)
     {
        println("IMU initialization unsuccessful");
        println("Check IMU wiring or try cycling power");
        println("Status: ");
        println(String(status));
        // while(1) {}
     }
    
     // init GPS
     gps.begin();

     // init pressure/temp
     pt.init();
    
     // init acoustic modem
     modem.begin();
    
     // init motors
     propulsion.begin();
     steering.begin();
     buoyancy.begin();
    
     // motor control task (pinned to CPU1 is CRITICAL for i2c communication)
     xTaskCreatePinnedToCore (
        afMotorsControlTask,                 /* Task function. */
        "motorControl",                   /* String with name of task. */
        1000,                             /* Stack size in words. */
        this,                          /* Parameter passed as input of the task */
        configMAX_PRIORITIES - 6,         /* Priority of the task. */
        NULL,                             /* Task handle. */
        1);                               /* Core where the task should run */
    
     // wait a bit before starting. Allows the user to reprogram in case his code is crashing...
     long t = millis();
     while (millis() - t < AF_INITIAL_WAIT_TIME)
     {
        ledstrip.setFront(Ledstrip::Rgbb(0, 255, 0, 31));
        delay(500);
        ledstrip.setFront(Ledstrip::Rgbb(0, 0, 0, 31));
        delay(500);
     }
}

void AFish2::enableGreenlight(bool enable)
{
    if (enable)        
        pwm.setPWM(AF_ENABLE_GREENLIGHT_PIN, 4096, 0); // on
    else
        pwm.setPWM(AF_ENABLE_GREENLIGHT_PIN, 0, 4096); // off
}

void AFish2::enableEsense(bool enable)
{
    if (enable)        
        pwm.setPWM(AF_ENABLE_ESENSE_PIN, 4096, 0); // on
    else
        pwm.setPWM(AF_ENABLE_ESENSE_PIN, 0, 4096); // off    
}

void AFish2::setBluetoothTerminalCallback(std::function<void(char*)> f)
{
    bluetoothTerminalCallback = f;
}

void AFish2::beginBluetoothTerminal()
{
    xTaskCreate(
        afBluetoothTerminalTask,          /* Task function. */
        "bluetoothTerminal",        /* String with name of task. */
        10000,            /* Stack size in words. */
        this,             /* Parameter passed as input of the task */
        1,                /* Priority of the task. */
        &bluetoothTerminalHandle);            /* Task handle. */
}

void AFish2::endBluetoothTerminal()
{
    vTaskDelete(bluetoothTerminalHandle);
}

size_t AFish2::write(uint8_t c)
{
 //    Serial.write(c);
    SerialBT.write(c);
    return 1;
}

size_t AFish2::write(const uint8_t *buffer, size_t size)
{
 //    Serial.write(buffer, size);
    SerialBT.write(buffer, size);
    return size;
}

void AFish2::sleep(long msec)
{}

void afBluetoothTerminalTask (void * parameter)
{
    // get af ptr
    AFish2* af = (AFish2*)parameter;
    
    while(1)
    {
        // incoming data
        if (af->SerialBT.available())
        {
            // record msg
            char buffer[256];
            int bufferSize = 0;
            while (af->SerialBT.available())
            {
                char c = af->SerialBT.read();
                if (c != 0 && c != '\n' && c!= '\r')
                {
                    buffer[bufferSize++] = c;
                }
                else
                {
                    buffer[bufferSize] = 0;
                    break;
                }                               

                if (!af->SerialBT.available())
                    delay(100);
            }
            
            // parse received message
            char delims[] = {' ', '\n', '\r', 0};
            char* tok = strtok (buffer, delims);
            if (tok != NULL)
            {
                
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // hello
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                
                if (strcmp(tok, "hello") == 0)
                {
                    af->SerialBT.print("hello to you !");
                   //                    Serial.print("reply : hello to you my friend...houba HOP 10!");
                }

                else if (strcmp (tok, "ota") == 0)
                {
                    tok = strtok(NULL, delims);

                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // ota bluetooth
                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                
                    if (strcmp (tok, "bluetooth") == 0)
                    {                    
                        char* md5 = strtok(NULL, delims);
                        int totalSize = atoi(strtok(NULL, delims));

                      //                   vTaskSuspendAll();
                        af->otaBluetooth(md5, totalSize);
                       //                 xTaskResumeAll();
                        
                        // returned from OTA, put front in red
                        af->ledstrip.setFront(Ledstrip::Rgbb(255, 0, 0, 31));
                    }

                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // ota wifi
                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                
                    if (strcmp (tok, "wifi") == 0)
                    {
                        char* wifiSsid = strtok(NULL, delims);
                        char* wifiPass = strtok(NULL, delims);
                        char* url = strtok(NULL, delims);
                        char* md5 = strtok(NULL, delims);
                        int totalSize = atoi(strtok(NULL, delims));

                        //                      vTaskSuspendAll();
                        af->otaWifi(wifiSsid, wifiPass, url, md5, totalSize);
                      //                        xTaskResumeAll();

                        // returned from OTA, put front in red
                        af->ledstrip.setFront(Ledstrip::Rgbb(255, 0, 0, 31));                        
                    }
                }
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                // pass to user callback 
                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                
                else
                {
                    if (af->bluetoothTerminalCallback)
                    {
                        // remove any NULL char put by previous strtok
                        for (int i = 0; i < bufferSize; i++)
                            if (buffer[i] == 0)
                                buffer[i] = ' ';

                        // pass buffer to user callback
                        af->bluetoothTerminalCallback(buffer);
                    }
                }
                
            } // if tok != NULL
        } // if SerialBT.available
        delay(500);
    } // while (1)
}


bool AFish2::otaWifi(char* wifiSsid, char* wifiPass, char* url, char* md5, int totalSize)
{    
    uint8_t buff[1024] = { 0 };           

    ledstrip.setFront(Ledstrip::Rgbb(128, 128, 0, 31));
        
    // we need to shutdown bluetooth to connect to wifi
    // not enough ram to run both stacks simultaneously...
    SerialBT.end();                        
    
    // init wifi                    
    WiFi.begin(wifiSsid, wifiPass);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    } 
                        
    HTTPClient http;
                        
    int res = http.begin(url);                        
    int httpCode = http.GET();
                        
    int downloaded = 0;
    WiFiClient* stream; // has te be declared here to comply with goto rules...
        
    if (httpCode <= 0 || httpCode != HTTP_CODE_OK)
        goto otaWifiFailed;
        
    // get length of document (is -1 when Server sends no Content-Length header)
    totalSize = http.getSize();
    
    // get tcp stream
    stream = http.getStreamPtr();
    
    if (!Update.begin(totalSize, U_FLASH))
        goto otaWifiFailed;    
    
    Update.setMD5(md5);
    
    // read all data from server
    while (!Update.isFinished())
    {
        // lost connection ?
        if (!http.connected())
        {
            goto otaWifiFailed;
        }
        
        // get available data size
        int size = stream->available();
        
        if (size > 0)
        {
            // read up to 128 byte
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
            
            // write to storage
            int written = Update.write(buff, c);
            if (written == c)
            {                                            
                downloaded += written;
            }
            else
            {
                goto otaWifiFailed;
            }
        }
        else
        {
            // lost connection ?
            int retries = 20;
            while (retries > 0)
            {
                if (!stream->connected())
                {
                    WiFi.reconnect();
                    retries--;
                }
                else
                {
                    break;
                }
                ledstrip.setFront(Ledstrip::Rgbb(255, 0, 0, 31));
                delay(200);
            }
            if (retries == 0)
                goto otaWifiFailed;

            // reconnected
            ledstrip.setFront(Ledstrip::Rgbb(128, 128, 0, 31));
        }
        delay(10);
    } // while update not finished
        
 //    http.end();
                        
    if(downloaded == totalSize)
    {
        if (Update.end(true))
        {
            delay(200);
            ESP.restart();                
        }
        else
            goto otaWifiFailed;
    }
    else
        goto otaWifiFailed;
                                               
    // restart bt connection, in case OTA failed
 otaWifiFailed:
    WiFi.disconnect(true);
    SerialBT.begin(name);
    return false;
}

bool AFish2::otaBluetooth(char* md5, int totalSize)
{
    ledstrip.setFront(Ledstrip::Rgbb(128, 128, 0, 31));
    
    if (Update.begin(totalSize, U_FLASH))
    {            
        Update.setMD5(md5);
                            
        // request to send data
        print("+");
                            
        // read all data from server
        long t = millis();
        int downloaded = 0;
        int written = 0;
        int packets = 0;
        int packetSize = 2048;
        uint8_t* buff = new uint8_t [packetSize];
        
        int i = 0;
        while (!Update.isFinished() && millis() - t < 5000)
        {                            
            // read data
            while (SerialBT.available() && i < packetSize)
            {
                buff[i] = SerialBT.read();
                i++;
            }
            
            // full packet or last packet
            if (i == packetSize || (downloaded > totalSize - packetSize && i > 0))
            {
                int w = Update.write(buff, i);
                if (w == i)
                {
                    written += w;
                    
                    if (written % (packetSize * 50) == 0)
                        print("+");
                }
                else
                {
                    // flashing failed
                    return false;
                }
                
                packets++;
                t = millis();
                downloaded += i;
                i = 0;
            }
        }
        
        if (written == totalSize)
        {
            if(Update.end(true))
            {
                delay(200);
                ESP.restart();                
            }
            else
            {
                return false;
                // Serial.println("Flashing md5 ... failed!"); 
            }
        }
    }
    return false;
}


// ----------------------------------------------------------------------------

void Gps::begin()
{
    Serial1.begin(9600, SERIAL_8N1, 17, 16);
};

bool Gps::read(int timeout)
{
    long t = millis();

    do
    {
        while (Serial1.available() > 0)
            if (encode(Serial1.read()))
                return true;
    } while (millis() - t < timeout);

    return false;
}
// ----------------------------------------------------------------------------

/* LoRa */

/* lora.sleep() */
/* lora.wakeup(){}; */

/* lora.send(int dest, char* msg) */
/* lora.receive() */

void Lora::begin(){};
void Lora::sleep(){};
void Lora::wakeup(){};
void Lora::send(int dest, char* msg, int size){};
int Lora::receive(char* msg, int maxSize){};

// ----------------------------------------------------------------------------

/* Inertial Measurement Unit */

/* imu.getCompass(float& x, float& y, float &z){}; */
/* imu.getGyro(float& x, float& y, float &z){}; */
/* imu.getAccel(float& x, float& y, float &z){}; */

//MPU9250 mpu9250 (Wire, 0x68);
/*
 void Imu::begin()
 {
     int status = IMU.begin();
     if (status < 0)
     {
         af.debugln("IMU beginialization unsuccessful");
         af.debugln("Check IMU wiring or try cycling power");
         af.debugln("Status: ");
         af.debugln(status);
         // while(1) {}
     }
 };
*/
// void Imu::getCompass(float& x, float& y, float& z){};
// void Imu::getGyro(float& x, float& y, float& z){};
// void Imu::getAccel(float& x, float& y, float& z){};       

// ----------------------------------------------------------------------------
// Pressure/Temperature sensor

void PressureTemperature::begin(int fluidDensity)
{
    setModel(MS5837::MS5837_30BA);
    setFluidDensity(fluidDensity); // kg/m^3 (freshwater, 1029 for seawater)
}  

// ----------------------------------------------------------------------------
// Sonar rangers


// ----------------------------------------------------------------------------
// Turbidity sensor

void Turbidity::begin(){};
float Turbidity::get(){};


// ----------------------------------------------------------------------------
// O2 sensor

void O2::begin(){};
void O2::sleep(){};
void O2::wakeup(){};
float O2::get(){};    

// ----------------------------------------------------------------------------
// PH sensor

void PH::begin(){};
void PH::sleep(){};
void PH::wakeup(){};
float PH::get(){};    

// ----------------------------------------------------------------------------
// Propulsion and steering motors

void Motor::begin()
{
    speed = 0;
}

void Motor::set(int speed)
{
    this->speed = speed;
}

void Motor::stop()
{
    speed = 0;
}

int Motor::get()
{
    return speed;
}

// ----------------------------------------------------------------------------
// Buoyancy system (motor, encoder, limit switches)
AiEsp32RotaryEncoder Buoyancy::encoder = AiEsp32RotaryEncoder(AF_ENCODER_A_PIN, AF_ENCODER_B_PIN, -1, -1);


Buoyancy::Buoyancy()
{

}

void Buoyancy::begin()
{
    // init motor
    Motor::begin();
    
    // init encoder
    encoder.begin();

//    std::function<void(void)> f = encoder.readEncoder_ISR;
//    encoder.setup(f);

    
//    encoder.setup([]{encoder.readEncoder_ISR();}); WILL NOT WORK BECAUSE WE ARE INSIDE CLASS, CAN@T CAPTURE THIS
    encoder.setup([]{encoder.readEncoder_ISR();});
    encoder.setBoundaries(-1000000, 1000000, true); //minValue, maxValue, cycle values (when max go to min and vice versa)
    encoder.enable();

    encoderUpperLimit = 1000000;
    encoderLowerLimit = -1000000;
}

// manually define current position
void Buoyancy::resetPosition(int newPosition)
{
    encoder.reset(newPosition);
}

int Buoyancy::getPosition()
{
    return position;
}

bool Buoyancy::reachedUpperLimit()
{
    return limitedUp;
}

bool Buoyancy::reachedLowerLimit()
{
    return limitedDown;
}

void Buoyancy::setUpperLimit(int l)
{
    encoderUpperLimit = l;
}

void Buoyancy::setLowerLimit(int l)
{
    encoderLowerLimit = l;
}

// ----------------------------------------------------------------------------
// Motors control task

void afMotorSetSpeed(AFish2* af, int motor, int speed)
{
    Adafruit_PWMServoDriver* pwm = &af->pwm;
    
    // first set motor pins
    int pinA = AF_PROPULSION_A_PIN;
    int pinB = AF_PROPULSION_B_PIN;
    int pinPwm = AF_PROPULSION_PWM_PIN;
    
    if (motor == AF_STEERING)
    {
        pinA = AF_STEERING_A_PIN;
        pinB = AF_STEERING_B_PIN;
        pinPwm = AF_STEERING_PWM_PIN;        
    }
    else if (motor == AF_BUOYANCY)
    {
        pinA = AF_BUOYANCY_A_PIN;
        pinB = AF_BUOYANCY_B_PIN;
        pinPwm = AF_BUOYANCY_PWM_PIN;        
    }

    // then send h-bridge control commands to gpio/pwm expander
    if (speed == 0)
    {
        pwm->setPWM(pinA, 0, 4096); // off
        pwm->setPWM(pinB, 0, 4096); // off
        pwm->setPWM(pinPwm, 0, 4096); // off
    }
    else if (speed > 0)
    {
        pwm->setPWM(pinA, 4096, 0); // on
        pwm->setPWM(pinB, 0, 4096); // off
        pwm->setPWM(pinPwm, 0, speed); // val
    }
    else // speed < 0
    {
        pwm->setPWM(pinA, 0, 4096); // off
        pwm->setPWM(pinB, 4096, 0); // on
        pwm->setPWM(pinPwm, 0, -speed); // val
    }
}

// task to implement motor control with i2c messages to gpio/pwm expander
void afMotorsControlTask(void* parameters)
{
    AFish2* af = (AFish2*) parameters;
    
    Adafruit_PWMServoDriver* pwm = &af->pwm;
    Switch* upperSwitch = &af->buoyancy.upperSwitch;
    //Switch* lowerSwitch = &af->buoyancy.lowerSwitch;
    AiEsp32RotaryEncoder* encoder = &af->buoyancy.encoder;
    
    // keep actual motor speed in memory, so we know when user makes a change
    int propulsionSpeed = 0;
    int steeringSpeed = 0;
    int buoyancySpeed = 0;

    int encoderPosition = 0;
    
    bool buoyancyLimitedUp = false;
    bool buoyancyLimitedDown = false;
    
    // fisrt call does not matter
    upperSwitch->poll();

    // main loop of motors control (should keep running at all times)
    while(1)
    {
        // 1. check limit switch status
        upperSwitch->poll();
        bool newLimitUp = false;
        bool newLimitDown = false;
        
        // if state has changed since last poll
       if (upperSwitch->switched())
        {
            // if limit switch was just pressed, disable motor immediately
            if (upperSwitch->on())
            {
                newLimitUp = true;
            }
        }
        // 2. check encoder position and see if limit reached
       encoderPosition = encoder->readEncoder();

        if (encoderPosition > af->buoyancy.encoderUpperLimit)
            newLimitUp = true;

        if (encoderPosition < af->buoyancy.encoderLowerLimit)
            newLimitDown = true;

        // 3. implement limitations on buoyancy        
        if (newLimitUp != buoyancyLimitedUp || newLimitDown != buoyancyLimitedDown)
        {            
            buoyancyLimitedUp = newLimitUp;
            buoyancyLimitedDown = newLimitDown;

            if (newLimitUp || newLimitDown)
            {
                buoyancySpeed = 0;
                afMotorSetSpeed(af, AF_BUOYANCY, buoyancySpeed);
            }
        }
        
        // 4. implement user control for propulsion and steering
        if (af->buoyancy.speed != buoyancySpeed)
        {
            if (
                (af->buoyancy.speed > 0 && !buoyancyLimitedUp) // go up
                || (af->buoyancy.speed < 0 && !buoyancyLimitedDown) // go down
                || (af->buoyancy.speed == 0) // stop
                )
            {
                buoyancySpeed = af->buoyancy.speed;
                afMotorSetSpeed(af, AF_BUOYANCY, buoyancySpeed);
            }
        }
        if (af->propulsion.speed != propulsionSpeed)
        {
            propulsionSpeed = af->propulsion.speed;
            afMotorSetSpeed(af, AF_PROPULSION, propulsionSpeed);
        }
        if (af->steering.speed != steeringSpeed)
        {
            steeringSpeed = af->steering.speed;
            afMotorSetSpeed(af, AF_STEERING, steeringSpeed);
        }
        
        // 5. update user structures
        af->propulsion.speed = propulsionSpeed;
        af->steering.speed = steeringSpeed;
        af->buoyancy.speed = buoyancySpeed;
        af->buoyancy.limitedUp = buoyancyLimitedUp;
        af->buoyancy.limitedDown = buoyancyLimitedDown;
        af->buoyancy.position = encoderPosition;
        
                
        // loop 20 times/sec
        delay(50);
    }

    vTaskDelete( NULL );
}


// ----------------------------------------------------------------------------
// Ledstrip

void Ledstrip::begin()
{
    digitalWrite(AF_LEDSTRIP_DATA_PIN, LOW);
    pinMode(AF_LEDSTRIP_DATA_PIN, OUTPUT);
    digitalWrite(AF_LEDSTRIP_CLOCK_PIN, LOW);
    pinMode(AF_LEDSTRIP_CLOCK_PIN, OUTPUT);
};


void Ledstrip::writeByte(uint8_t b)
{
    uint8_t pos;
    for (pos = 0; pos <= 7; pos++)
    {
        digitalWrite(AF_LEDSTRIP_DATA_PIN, b >> (7-pos) & 1);
        digitalWrite(AF_LEDSTRIP_CLOCK_PIN, HIGH);
        digitalWrite(AF_LEDSTRIP_CLOCK_PIN, LOW);
    }
}

void Ledstrip::startFrame()
{
    writeByte(0);
    writeByte(0);
    writeByte(0);
    writeByte(0);
}

void Ledstrip::endFrame()
{
    writeByte(0);
    writeByte(0);
    writeByte(0);
    writeByte(0);
}

void Ledstrip::write(Rgbb color)
{
    writeByte(0b11100000 | color.brightness);
    writeByte(color.b);
    writeByte(color.g);
    writeByte(color.r);
}

void Ledstrip::writeColors(Rgbb* colors, int count)
{
    // first check that power consumption is within limits
    int totalPower = 0;
    for (int i = 0; i < count; i++)
    {
        int sum = colors[i].r + colors[i].g + colors[i].b;        
        totalPower += sum * colors[i].brightness;
    }

    // user request too much power, discard command
    if (totalPower > maxPower)
        return;
        
    startFrame();
    for(int i = 0; i < count; i++)
    {
        write(colors[i]);
    }
    endFrame();
} 

Ledstrip::Rgbb Ledstrip::hsvToRgb(Hsvb hsv)
{
    Rgbb rgb;
    rgb.brightness = hsv.brightness;
    
    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    unsigned char region = hsv.h / 43;
    unsigned char remainder = (hsv.h - (region * 43)) * 6; 

    unsigned char p = (hsv.v * (255 - hsv.s)) >> 8;
    unsigned char q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    unsigned char t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }
    
    return rgb;
}

void Ledstrip::writeColors(Hsvb* colors, int count)
{
    // first convert hsv to rgb
    Rgbb rgbColors[count];

    for (int i = 0; i < count; i++)
        rgbColors[i] = hsvToRgb(colors[i]);

    // next try to display
    writeColors(rgbColors, count);
} 

// maxPower expressed here in milliamps !!!
void Ledstrip::setMaxPower(int maxPower)
{
    // convert from mA to internal units
    // one led consumes max 255 * 3 * 31 "units", equivalent to 60mA
    this->maxPower = (maxPower * 255 * 3 * 31) / 60;
}

void Ledstrip::off()
{
    Rgbb c;
    setAll(c);
}

void Ledstrip::setAll(Rgbb c)
{
    // check power consumption
    int oneled = (c.r + c.g + c.b) * c.brightness;
    int totalPower = oneled * AF_LEDSTRIP_NUM_LEDS;

    int step = 1;
    
    if (totalPower > maxPower)
    {        
        if (totalPower / 2 <= maxPower)
            // reduce resolution to comply with max power
            step = 2;
        else
            // can't comply
            return;
    }

    // write all leds
    startFrame();
    for(int i = 0; i < AF_LEDSTRIP_NUM_LEDS; i += step)
    {
        write(c);
    }
    endFrame();
}

void Ledstrip::setAll(Hsvb hsv)
{
    Rgbb rgb = hsvToRgb(hsv);
    setAll(rgb);
}

void Ledstrip::setFront(Rgbb col)
{
    Rgbb black;
    
    // no max power check, this should be ok (< 500mA)
    startFrame();
    for(int i = 0; i < AF_LEDSTRIP_NUM_LEDS - AF_LEDSTRIP_NUM_LEDS_FRONT; i++)
    {
        write(black);
    }
    for(int i = AF_LEDSTRIP_NUM_LEDS - AF_LEDSTRIP_NUM_LEDS_FRONT; i < AF_LEDSTRIP_NUM_LEDS; i++)
    {
        write(col);
    }
    endFrame();    
}

void Ledstrip::setRight(Rgbb col)
{
    Rgbb black;
    
    // no max power check, this should be ok (< 500mA)
    startFrame();
    for(int i = 0; i < AF_LEDSTRIP_NUM_LEDS_RIGHT; i++)
    {
        write(col);
    }
    for(int i = AF_LEDSTRIP_NUM_LEDS_RIGHT; i < AF_LEDSTRIP_NUM_LEDS; i++)
    {
        write(black);
    }
    endFrame();    
}

void Ledstrip::setLeft(Rgbb col)
{
    Rgbb black;

    int start = AF_LEDSTRIP_NUM_LEDS_RIGHT + 1;
    int end = AF_LEDSTRIP_NUM_LEDS_RIGHT + 1 + AF_LEDSTRIP_NUM_LEDS_LEFT;
    
    // no max power check, this should be ok (< 500mA)
    startFrame();
    for(int i = 0; i < start; i++)
    {
        write(black);
    }
    for(int i = start; i < end; i++)
    {
        write(col);
    }
    for(int i = end; i < AF_LEDSTRIP_NUM_LEDS; i++)
    {
        write(black);
    }
    endFrame();    
}

void Ledstrip::setFront(Hsvb hsv)
{
    Rgbb rgb = hsvToRgb(hsv);
    setFront(rgb);
}

void Ledstrip::setRight(Hsvb hsv)
{
    Rgbb rgb = hsvToRgb(hsv);
    setRight(rgb);
}

void Ledstrip::setLeft(Hsvb hsv)
{
    Rgbb rgb = hsvToRgb(hsv);
    setLeft(rgb);
}

              
// ----------------------------------------------------------------------------

void Modem::begin()
{
    Serial.begin(9600, SERIAL_8N1, 1, 3); // inverted tx/rx pins
//    Serial.begin(9600);
}

int Modem::available()
{
    return Serial.available();
}

int Modem::receive(char* buffer)
{
    if (Serial.available())
    {
        receivedBroadcast = false;
        senderId = -1;
        char modemBuffer[16];
        int len = 0;
        
        while (Serial.available())
        {
            while (Serial.available())
            {
                modemBuffer[len++] = Serial.read();
                delay(10);
            }
            delay(500);
        }
        
        if (modemBuffer[0] == '#')
        {
            // command received
            if (modemBuffer[1] == 'U')
            {            
                len = modemBuffer[2];
                
                for (int i = 0; i < len; i++)
                {
                    modemBuffer[i] = modemBuffer[i+3];
                }
                
                memcpy(buffer, modemBuffer, len);
                buffer[len] = 0;
                
                return len;
            }
            else if (modemBuffer[1] == 'B')
            {
                int d100 = modemBuffer[2] - '0';
                int d10 = modemBuffer[3] - '0';
                int d1 = modemBuffer[4] - '0';

                receivedBroadcast = true;
                senderId = d100 * 100 + d10 * 10 + d1;
            
                len = modemBuffer[5];
                for (int i = 0; i < len; i++)
                {
                    modemBuffer[i] = modemBuffer[i+6];
                }

                memcpy(buffer, modemBuffer, len);
                buffer[len] = 0;                
                
                return len;
            }
        }
    }
}

int Modem::getStatus(char* buffer)
{
    char modemBuffer[16];
    int len = 0;

    // send $?
    modemBuffer[0] = '$';
    modemBuffer[1] = '?';
    Serial.write((uint8_t*)modemBuffer, 2);
    
    delay(100);

    // read reply from modem
    while (Serial.available())
    {
        while (Serial.available())
        {
            modemBuffer[len++] = Serial.read();
            delay(10);
        }
        delay(500);
    }
        
    if (modemBuffer[0] == '#')
    {
        if (modemBuffer[1] == 'A')
        {
            int i = 0;
            modemBuffer[i++] = modemBuffer[2];
            modemBuffer[i++] = modemBuffer[3];
            modemBuffer[i++] = modemBuffer[4];
            modemBuffer[i++] = modemBuffer[6];
            modemBuffer[i++] = modemBuffer[7];
            modemBuffer[i++] = modemBuffer[8];
            modemBuffer[i++] = modemBuffer[9];
            
            memcpy(buffer, modemBuffer, 7);
            buffer[7] = 0;
            return 7;            
        }
    }

    return 0;
}

void Modem::send(int dest, String msg)
{
    char buffer[16];

    int size = msg.length();
    if (size > 7) size = 7;

    int i = 0;
    buffer[i++] = '$';
    buffer[i++] = 'U';
    buffer[i++] = dest / 100 + '0';
    buffer[i++] = (dest / 10) % 10 + '0';
    buffer[i++] = dest % 10 + '0';

    buffer[i++] = size + '0';
        
    for (int j = 0; j < size; j++)
        buffer[i++] = msg.charAt(j);

    buffer[i] = 0;

    // buffer is ready, send to modem
    Serial.print(buffer);

    // wait that msg is transmitted
    delay (300 * size);
}

void Modem::ping(int dest)
{
    char buffer[16];
    
    int i = 0;
    buffer[i++] = '$';
    buffer[i++] = 'P';
    buffer[i++] = dest / 100 + '0';
    buffer[i++] = (dest / 10) % 10 + '0';
    buffer[i++] = dest % 10 + '0';

    buffer[i] = 0;

    // buffer is ready, send to modem
    Serial.print(buffer);

    // wait that msg is transmitted
    delay (300 * i);
}

void Modem::broadcast(int dest, String msg)
{
    char buffer[16];

    int size = msg.length();
    if (size > 7) size = 7;

    int i = 0;
    buffer[i++] = '$';
    buffer[i++] = 'B';
    buffer[i++] = size + '0';
        
    for (int j = 0; j < size; j++)
        buffer[i++] = msg.charAt(j);

    buffer[i] = 0;

    // buffer is ready, send to modem
    Serial.print(buffer);

    // wait that msg is transmitted
    delay (300 * size);
}
