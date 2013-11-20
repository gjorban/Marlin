// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
<<<<<<< HEAD

#ifndef MARLIN_H
#define MARLIN_H

#define  HardwareSerial_h // trick to disable the standard HWserial

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include  <avr/wdt.h>
#include  <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#if ARDUINO >= 100 
  #if defined(__AVR_ATmega644P__)
    #include "WProgram.h"
  #else
    #include "Arduino.h"
  #endif
#else
   #include "WProgram.h"
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#if MOTHERBOARD == 8  // Teensylu
  #define MYSERIAL Serial
#else
  #define MYSERIAL MSerial
#endif

//this is a unfinsihed attemp to removes a lot of warning messages, see:
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=57011
//typedef char prog_char PROGMEM; 
// //#define PSTR    (s )        ((const PROGMEM char *)(s))
// //# define MYPGM(s) (__extension__({static prog_char __c[] = (s); &__c[0];})) 
// //#define MYPGM(s) ((const prog_char *g PROGMEM=s))
#define MYPGM(s) PSTR(s)
//#define MYPGM(s)  (__extension__({static char __c[] __attribute__((__progmem__)) = (s); &__c[0];}))  //This is the normal behaviour
//#define MYPGM(s)  (__extension__({static prog_char __c[]  = (s); &__c[0];})) //this does not work but hides the warnings


#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(MYPGM(x));
#define SERIAL_PROTOCOLLN(x) {MYSERIAL.print(x);MYSERIAL.write('\n');}
#define SERIAL_PROTOCOLLNPGM(x) {serialprintPGM(MYPGM(x));MYSERIAL.write('\n');}


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) {SERIAL_ECHOPGM(name);SERIAL_ECHO(value);}


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
#define SerialprintPGM(x) serialprintPGM(MYPGM(x))
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}


void get_command();
void process_commands();

void manage_inactivity(byte debug);

#if X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if Z_ENABLE_PIN > -1
  #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
  #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

=======
#include <WProgram.h>
#include "fastio.h"
extern "C" void __cxa_pure_virtual();
void __cxa_pure_virtual(){};
void get_command();
void process_commands();

void manage_inactivity(byte debug);

void manage_heater();
int temp2analogu(int celsius, const short table[][2], int numtemps);
float analog2tempu(int raw, const short table[][2], int numtemps);
#ifdef HEATER_USES_THERMISTOR
    #define HEATERSOURCE 1
#endif
#ifdef BED_USES_THERMISTOR
    #define BEDSOURCE 1
#endif

#define temp2analogh( c ) temp2analogu((c),temptable,NUMTEMPS)
#define analog2temp( c ) analog2tempu((c),temptable,NUMTEMPS)

#if X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif
#if Y_ENABLE_PIN > -1
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif
#if Z_ENABLE_PIN > -1
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
#define enable_z() ;
#define disable_z() ;
#endif
#if E_ENABLE_PIN > -1
#define  enable_e() WRITE(E_ENABLE_PIN, E_ENABLE_ON)
#define disable_e() WRITE(E_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e() ;
#define disable_e() ;
#endif

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
>>>>>>> c9005865ed8ebacfa7bf09303be39288ecd882cd

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
<<<<<<< HEAD
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void prepare_arc_move(char isclockwise);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern unsigned char FanSpeed;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#endif
=======
void linear_move(unsigned long steps_remaining[]);
void do_step(int axis);
void kill(byte debug);

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  long step_event_count;                    // The number of step events required to complete this block
  volatile long accelerate_until;                    // The index of the step event on which to stop acceleration
  volatile long decelerate_after;                    // The index of the step event on which to start decelerating
  volatile long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  long advance_rate;
  volatile long initial_advance;
  volatile long final_advance;
  float advance;

  // Fields used by the motion planner to manage acceleration
  float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/minute for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/min  
  float millimeters;                                 // The total travel of this block in mm
  float entry_speed;
  float acceleration;                                // acceleration mm/sec^2

  // Settings for the trapezoid generator
  long nominal_rate;                                 // The nominal step rate for this block in step_events/sec 
  volatile long initial_rate;                        // The jerk-adjusted step rate at start of block  
  volatile long final_rate;                          // The minimal rate at exit
  long acceleration_st;                              // acceleration steps/sec^2
  volatile char busy;
} block_t;

void check_axes_activity();
void plan_init();
void st_init();
void tp_init();
void plan_buffer_line(float x, float y, float z, float e, float feed_rate);
void plan_set_position(float x, float y, float z, float e);
void st_wake_up();
void st_synchronize();

>>>>>>> c9005865ed8ebacfa7bf09303be39288ecd882cd
