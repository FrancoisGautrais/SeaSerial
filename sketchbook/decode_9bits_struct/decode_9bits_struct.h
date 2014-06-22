/*
  decode_9bits_struct.h - SeaSerial library, to encapsulate Seatalk Datagrams
  By François Gautrais June 2014

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
* For more information about SeaTalk protocol please visit:
* http://www.thomasknauf.de/seatalk.htm
* (by Thomas Knauf)
*/


#ifndef DECODE_9BITS_STRUCT_H
#define DECODE_9BITS_STRUCT_H


struct auto_param_t
{
    //autopilot parameters
  volatile uint8_t rudder_gain; // [1-9]
  volatile uint8_t counter_rudder; // [1-9]
  volatile uint8_t rudder_limit; // [10-40]
  volatile uint8_t turn_rate_speed; // [1-30]
  volatile uint8_t speed; // [4-60]
  volatile uint8_t off_course_limit; // [15-40]
  volatile uint8_t trim; // [0-4]
  volatile uint8_t power_steer; // On/OFF
  volatile uint8_t drive_type; // {3,4,5} 
  volatile uint8_t rudder_damping; //[1-9]
  volatile int8_t variations;// [-30-30]
  volatile uint8_t auto_adapt; // 0=Off, 1=North, 2=South
  volatile uint8_t auto_adapt_latitude; // [0-80]
  volatile bool auto_release; //on/off
  volatile int8_t rudder_alignement; // [-7-7]
  volatile uint8_t wind_trim; // [1-9]
  volatile uint8_t response; // [1-9]
  volatile uint8_t boat_type; //1=disp,2=semi-displ,3=plan,4=stern,5=work,6=sail
  
  
  
  volatile bool cal_lock; //on/off
  volatile uint8_t tack_angle; //[40-125   
};

struct data_t
{
  volatile float truewindspeed; //knots
  volatile float apparentwindspeed; //knots
  volatile float truewindangle; //degres
  volatile float apparentwindangle; //knots/degres
  volatile float depthwater; //meter
  volatile float waterspeed; //knots
  volatile float groundspeed; //knots
  volatile float angle; // degres
  volatile float trip, total; // in Nautile Milles
  volatile float watertemp; //celsius
  volatile float speeds1, speeds2; //knots
  volatile uint8_t date_y,date_m,date_d; //dd/mm/yy
  volatile uint8_t time_h, time_m, time_s; //GMT
  volatile uint8_t lamp_intensity; //0,1,2 or 4
  volatile bool mob_disable; //man over board
  volatile uint8_t alarms; //various alarms
  volatile uint8_t wind_alarms; //alarms on true/apparent wind, 
                                // angle/speed, high or low
                                
  volatile float raw_latitude, raw_longitude; //degres
  volatile float latitude, longitude; //degres
  volatile float magnetic_course; // degres
  volatile uint8_t n_sat; //number of satelites
  volatile float compass_heading; //degres
  volatile bool rudder_turning; //true: right, false: left
  volatile bool turning; //true: right, false: left
  volatile uint8_t autopilot_mode; // &1: automode &2: standbymode  &4: vane mode, &8: trackmod
  volatile float autopilot_course;
  volatile int rudder_direction; //degres (negatif: left) (positif: right)
  volatile int rudder_gain;
  volatile float stear_reference;
  volatile bool st40_mode;//1=AUto mode, 0 locked mode
  volatile bool temp_sensor_error; //
  volatile uint8_t mil_sp_unit;
  auto_param_t autopilot;
  //volatile uint8_t
   
};


float km_to_nm(float km)
{
  return km/1.852;
}

float kmh_to_knots(float kmh)
{
  return kmh/1.852;
}

float mph_to_knots(float mph)
{
  return mph/1.15078;
}

float mi_to_nm(float mph)
{
  return mph/1.15078;
}

//on alarms
#define ALARM_ANCHOR_ALARAM (1<<0)
#define ALARM_METRIC_DISPLAY (1<<1)
#define ALARAM_UNUSED (1<<2)
#define ALARM_TRANSDUCTER_DEFECTIVE (1<<3)
#define ALARM_DEPTH (1<<4)
#define ALARM_SHALLOW_DEPTH (1<<5)
#define ALARM_OFF_COURSE (1<<6)
#define ALARM_WIND_SHIFT (1<<7)

//on mil_sp_unit
#define UNIT_NM 0
#define UNIT_SM 1
#define UNIT_KM 2


//on wind_alarms
#define WIND_ALARM_APPARENT_ANGLE_LOW (1<<7)
#define WIND_ALARM_APPARENT_ANGLE_HIGH (1<<6)
#define WIND_ALARM_APPARENT_SPEED_LOW (1<<5)
#define WIND_ALARM_APPARENT_SPEED_HIGH (1<<4)
#define WIND_ALARM_TRUE_ANGLE_LOW (1<<3)
#define WIND_ALARM_TRUE_ANGLE_HIGH (1<<2)
#define WIND_ALARM_TRUE_SPEED_LOW (1<<1)
#define WIND_ALARM_TRUE_SPEED_HIGH (1<<0)


//boat_type (autopilot)
#define BOAT_TYPE_DISPL 1
#define BOAT_TYPE_SEM_DISPL 2
#define BOAT_TYPE_PLAN 3
#define BOAT_TYPE_STERN 4
#define BOAT_TYPE_WORK 5
#define BOAT_TYPE_SAIL 6


#endif

