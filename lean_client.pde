// lean_client.pde

/*  Built for TinyDuino with
 *   - SI4432 radio transceiver
 *   - BMA250 3-axis accelerometer
 *  
 *  Calibrate accelerometer first:
 *   - Place on level spot and update values in read_accel() below (these could be variables but I'm lazy)
 *   - Place in 1.000 SG water at 25º C
 *   - Calibrate with https://github.com/universam1/iSpindel/blob/master/docs/Kalibrierung_en.xlsm
 *  
 */

#include <RHReliableDatagram.h>
#include <RH_RF22.h>
#include <SPI.h>
#include <Wire.h>         // for I2C communication with sensor
#include <BMA250.h>
#include <SimpleKalmanFilter.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#if defined(ARDUINO_ARCH_SAMD)
#define SerialMonitor SerialUSB
#else
#define SerialMonitor Serial
#endif
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// set CPU to 1MHz
#define F_CPU 1000000
// number of deep sleep cycles (~7-8 seconds per iteration)
#define ITERATIONS 450            // 1 hour
//#define ITERATIONS 1

//uncomment for debugging
//#define DEBUG true

BMA250 accel_sensor;

// singleton instance of the radio driver
RH_RF22 driver(7, 3);

// class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

SimpleKalmanFilter filter(0.5, 0.5, 0.01);

float x, y, z, pitch;
int temp;

void setup()
{
#ifdef DEBUG
  SerialMonitor.begin(9600);
  while (!SerialMonitor); // on TinyScreen+/SAMD21 platform, this will wait until the Serial Monitor is opened or until 5 seconds has passed
  SerialMonitor.print("!!!");
#endif

  // set all pins to OUTPUT, LOW
   for (byte i = 0; i <= A5; i++)
    {
    pinMode (i, OUTPUT);    // changed as per below
    digitalWrite (i, LOW);  //     ditto
    }

  // setup the BMA250 accelerometer sensor
  accel_sensor.begin(BMA250_range_2g, BMA250_update_time_64ms);
  // setup the SI SI4432 transmitter
  manager.init();
  
  // defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
#ifdef DEBUG  
  SerialMonitor.println("!!!");
#endif
  // wait 5 minutes before taking the first sample
  delay(300000);
}

ISR (WDT_vect) {
  wdt_disable(); // disable watchdog
}

// dont put this on the stack:
uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];
uint8_t data[RH_RF22_MAX_MESSAGE_LEN];

void loop()
{
#ifdef DEBUG
  SerialMonitor.println("---- loop start ----");
#endif
  uint8_t ReadBuff[6]; 
  
  update_pitch();
  
  int batt = getBandgap(); // determines what actual vcc is (*100), based on known bandgap voltage
#ifdef DEBUG
  SerialMonitor.println("------ batt ------");
  SerialMonitor.print("-------- voltage: "); SerialMonitor.println(batt);
  SerialMonitor.println("------ send start");
  SerialMonitor.print("-------- rf status: "); SerialMonitor.println(driver.statusRead());
  SerialMonitor.print("-------- rf mode: "); SerialMonitor.println(driver.mode());
  SerialMonitor.print("-------- rf temperature: "); SerialMonitor.println(driver.temperatureRead() * 0.5);
#endif

#define frac(x)     (int(1000*(x - int(x))))
 
  // send data in json format for easier consumption 
  sprintf(data, "{\"b\":%02d,\"p\":%d.%03d,\"t\":%d}", batt, int(pitch), abs(frac(pitch)), int((temp * 1.8)+32));
  manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS);
  driver.sleep();
  
#ifdef DEBUG
  SerialMonitor.println("------ send complete");
  SerialMonitor.println("---- loop end ----");
#endif

  for ( int i=0; i <= ITERATIONS; i++ ){
    update_pitch();
    deep_sleep();
  }
}

float update_pitch() {
  read_accel();
  // pitch = -atan(x / sqrt(y * y + z * z)) * 1 / (PI / 180);
  pitch = filter.updateEstimate(-atan(x / sqrt(y * y + z * z)) * 1 / (PI / 180));
  // pitch is relative to the x-axis, but for calculating ºP we need to know the angle of the y-axis
  pitch = 90 - pitch;
  #ifdef DEBUG
  SerialMonitor.print("---------- pitch: "); SerialMonitor.println(pitch);
  #endif
}

void read_accel() {
  accel_sensor.read();
  // place accelerometer on a level spot, adjust the below offsets to 0.0 while orienting in each direction
  x = accel_sensor.X - 8;
  y = accel_sensor.Y + 8;
  z = accel_sensor.Z - 7;
  temp = ((accel_sensor.rawTemp * 0.5) + 24.0);
}

void deep_sleep() {
#ifdef DEBUG
  SerialMonitor.flush(); 
#endif
  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and interval
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0); // 8 second delay (max)
  wdt_reset();

  // disable ADC
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  // turn off various modules
  byte old_PRR = PRR;
  PRR = 0xFF;
  power_all_disable(); 

  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts();
  
  sleep_cpu();
  
  sleep_disable();
  PRR = old_PRR;
  ADCSRA = old_ADCSRA;
  power_all_enable();
}

int getBandgap(void) // returns actual value of vcc*100
{

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // for mega boards
  const long InternalReferenceVoltage = 1115L;  // adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0               --> 0 1, AVcc internal ref.  -selects AVcc reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -selects channel 30, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#else
  // for 168/328 boards
  const long InternalReferenceVoltage = 1056L;  // adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#endif
  delay(50);  // let mux settle a little to get a more stable A/D conversion
  // start a conversion
  ADCSRA |= _BV( ADSC );
  // wait for it to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
  // scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
  return results;

}
