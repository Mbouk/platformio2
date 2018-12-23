

#ifndef AFISH2_H
#define AFISH2_H

#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
//#include "Wire.h"

#include "BluetoothSerial.h"
//BluetoothSerial SerialBT;

#define AF_ENCODER_A_PIN 27
#define AF_ENCODER_B_PIN 36
#define AF_UPPER_SWITCH_PIN 26
#define AF_LOWER_SWITCH_PIN -1

#define AF_INITIAL_WAIT_TIME 5000

#define AF_BUOYANCY_A_PIN 4
#define AF_BUOYANCY_B_PIN 5
#define AF_BUOYANCY_PWM_PIN 6

#define AF_PROPULSION_A_PIN 8
#define AF_PROPULSION_B_PIN 9
#define AF_PROPULSION_PWM_PIN 10

#define AF_STEERING_A_PIN 12
#define AF_STEERING_B_PIN 13
#define AF_STEERING_PWM_PIN 14

#define AF_PROPULSION 0
#define AF_STEERING 1
#define AF_BUOYANCY 2

#define AF_LEDSTRIP_CLOCK_PIN 14
#define AF_LEDSTRIP_DATA_PIN 13
#define AF_LEDSTRIP_NUM_LEDS 22
#define AF_LEDSTRIP_NUM_LEDS_FRONT 5
#define AF_LEDSTRIP_NUM_LEDS_LEFT 8
#define AF_LEDSTRIP_NUM_LEDS_RIGHT 8

#define AF_ENABLE_GREENLIGHT_PIN 0
#define AF_ENABLE_ESENSE_PIN 1

class AFish2;

void afBluetoothTerminalTask (void * parameter);
void afMotorsControlTask(void* parameters);
void afMotorSetSpeed(AFish2* af, int motor, int speed);

// ======================================================================

#include <TinyGPS++.h>

class Gps : public TinyGPSPlus 
{
public:
    // use parent constructor
    using TinyGPSPlus::TinyGPSPlus;

    void begin();
    bool read(int timeout = 0);
};

// ======================================================================

/* LoRa */

/* lora.sleep() */
/* lora.wakeup(); */

/* lora.send(int dest, char* msg) */
/* lora.receive() */

class Lora
{
public:
    Lora() {}

    void begin();
    void sleep();
    void wakeup();
    void send(int dest, char* msg, int size);
    int receive(char* msg, int maxSize);
};

// ======================================================================

/* Inertial Measurement Unit */

/* imu.getCompass(float& x, float& y, float &z); */
/* imu.getGyro(float& x, float& y, float &z); */
/* imu.getAccel(float& x, float& y, float &z); */

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
//extern MPU9250 imu;

class Imu : public MPU9250
{
public:
//    Imu() {}
    using MPU9250::MPU9250;
    /* void begin(); */
    /* void getCompass(float& x, float& y, float& z); */
    /* void getGyro(float& x, float& y, float& z); */
    /* void getAccel(float& x, float& y, float& z);        */
};

// ======================================================================

/* Pressure sensor */

#include "MS5837.h"

class PressureTemperature : public MS5837
{
public :
    using MS5837::MS5837;

    void begin(int fluidDensity = 1029); // kg/m^3 (997 for freshwater, 1029 for seawater)    
};  

// ======================================================================
// Sonar rangers
/*
#include "MaxSonarI2C.h"

class Sonar : public MaxSonarI2C
{
public:
    using MaxSonarI2C::MaxSonarI2C;
    
};
*/
// ======================================================================
// Turbidity sensor

class Turbidity
{
public:
    Turbidity() {};
    void begin();

    float get();
};

// ======================================================================
// O2 sensor

class O2
{
public:
    O2() {};
    void begin();

    void sleep();
    void wakeup();
    float get();    
};

// ======================================================================
// PH sensor

class PH
{
public:
    PH() {};
    void begin();

    void sleep();
    void wakeup();
    float get();    
};

// ======================================================================
// Propulsion and rotation motors

class Motor
{
public:
    int speed = 0;
    
    Motor() {};
    void begin();

    void set(int speed);
    void stop();
    int get();
};

// ======================================================================
// Buoyancy motor control

#include "AiEsp32RotaryEncoder.h"
#include "avdweb_Switch.h"

class Buoyancy : public Motor
{
public:
//    static AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(AF_ENCODER_A_PIN, AF_ENCODER_B_PIN, -1, -1);
    static AiEsp32RotaryEncoder encoder;
    Switch upperSwitch = Switch(AF_UPPER_SWITCH_PIN, INPUT_PULLUP, LOW);
//    Switch lowerSwitch = Switch(AF_LOWER_SWITCH_PIN, INPUT_PULLUP, LOW);

    int encoderUpperLimit;
    int encoderLowerLimit;

    bool limitedUp = false;
    bool limitedDown = false;
    int position;
    
    Buoyancy();

    void begin();

    void resetPosition(int newPos);
    int getPosition();

    void setUpperLimit(int l);
    void setLowerLimit(int l);

    bool reachedUpperLimit();
    bool reachedLowerLimit();
};


// ======================================================================
// Ledstrip


// 17 leds
// 5 in nose

class Ledstrip
{
public:

    struct Rgbb
    {
        unsigned char r, g, b, brightness;

        Rgbb()
            {
                r = 0;
                g = 0;
                b = 0;
                brightness = 0;
            }
        
        Rgbb(unsigned char r, unsigned char g, unsigned char b, unsigned char brightness)
            {
                this->r = r;
                this->g = g;
                this->b = b;
                this->brightness = brightness;
            }        
    };

    struct Hsvb
    {
        unsigned char h, s, v, brightness;

        Hsvb()
            {
                h = 0;
                s = 0;
                v = 0;
                brightness = 0;
            }
        
        Hsvb(unsigned char h, unsigned char s, unsigned char v, unsigned char brightness)
            {
                this->h = h;
                this->s = s;
                this->v = v;
                this->brightness = brightness;
            }        
    };
    
    int maxPower = 31*255*3*10; // max 13 leds fully bright, 60mA per led
    
    Ledstrip() {};

    void begin();
    Rgbb hsvToRgb(Hsvb hsv);
    void writeColors(Hsvb* colors, int count);
    void setMaxPower(int maxPower);

    void off();
    
    void setAll(Rgbb c);
    void setAll(Hsvb hsv);
    
    void setFront(Rgbb col);
    void setRight(Rgbb col);
    void setLeft(Rgbb col);
    
    void setFront(Hsvb hsv);
    void setRight(Hsvb hsv);
    void setLeft(Hsvb hsv);

private:
    // private because no safety check here
    void writeByte(uint8_t b);
    void startFrame();
    void endFrame();
    void write(Rgbb color);
    void writeColors(Rgbb* colors, int count);
};

// ======================================================================

/* Acoustic modem */

class Modem
{
public:
//    char buffer[16];
    bool receivedBroadcast;
    int senderId;
    
    Modem(){};

    void begin();

    int available();
    int receive(char* buffer);
    void send(int dest, String msg);
    void ping(int dest);    
    void broadcast(int dest, String msg);
    int getStatus(char* buffer);
};


// ======================================================================

#include <Adafruit_PWMServoDriver.h>


class AFish2 : public Print
{
public:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Members
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // devices
    Gps gps;
    Lora lora;
    Imu imu = Imu (Wire, 0x68);
    PressureTemperature pt; // p for pressure, t for temperature
 //   Sonar sonarFront = Sonar(0x30); 
//    Sonar sonarBottom = Sonar(0x32);
    Turbidity turbidity;
    O2 o2;
    PH pH;
    Motor propulsion;
    Motor steering;
    Buoyancy buoyancy;
    Ledstrip ledstrip;    
    BluetoothSerial SerialBT;
    Modem modem;    
    
    // internal stuff
    int id;
    char* name;
   std::function<void(char*)> bluetoothTerminalCallback;

    Adafruit_PWMServoDriver pwm;
           
    // task to handle basic commands
    TaskHandle_t bluetoothTerminalHandle = NULL;
    TaskHandle_t motorsControlHandle = NULL;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Methods
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // empty constructor. Init is decide by user
    AFish2()
    //        : imu (Wire, 0x68),
        {};
        
    void begin(int id, char* name);
    void sleep(long msec);

    // debug
    void beginBluetoothTerminal();
    void endBluetoothTerminal();
    void setBluetoothTerminalCallback(std::function<void(char*)>);

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    bool otaWifi(char* ssid, char* pass, char* url, char* md5, int totalSize);
    bool otaBluetooth(char* md5, int totalSize);

    void enableGreenlight(bool enable);
    void enableEsense(bool enable);

private :
    // no safety for these internal functions, please use public stuff
//    void motorSetSpeed(int motor, int speed);
//    void motorsControlTask(void* parameters);

};

#endif
