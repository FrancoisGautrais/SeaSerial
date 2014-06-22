
/*
  decode_9bits_struct.ino - SeaSerial library, to encapsulate Seatalk Datagrams
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
* Decode some SeaTalk Frames and store result in a structure
*
* For more information about SeaTalk protocol please visit:
* http://www.thomasknauf.de/seatalk.htm
* (by Thomas Knauf)
*/

#include <SeaSerial.h>
#include "decode_9bits_struct.h"

char mem[30];
int i=0;

SeaSerial serial;
data_t data={0};

void setup()
{
        
	;//Serial.begin(115200);
        pinMode(13,OUTPUT);
        while (!Serial); 
}

void print_float(const char* bef, double x,const  char* unity, const char* end=NULL)
{
  Serial.write(bef);
  Serial.print(x,5);
  Serial.write(unity);
  if(end) Serial.write(end);
}

void loop()
{
  decode(data,serial);
}


int decode(data_t& data, SeaSerial& serial)
{	
  unsigned long t1,t2;
  
  static double av=0.0;
  static int nb=0;
  
  t1=micros();
  
  
  serial.next_frame();
   switch(serial.get_cmd())
   {
     case 0x00:
       if(serial.get_4(3)==2)
       {
         uint16_t x,y,z;
         y=z=serial.get_8(2);
         y=(y>>4);
         z=(z&0xf);
         x=serial.get_int(3,4);
         if(y&0b1000) data.alarms|=ALARM_ANCHOR_ALARAM; ;//Serial.write("Anchor Alarm is active\n");
         if(y&0b0100) data.alarms|=ALARM_METRIC_DISPLAY;//Serial.write("Metric display units \n");
         if(y&0b0010) data.alarms|=ALARAM_UNUSED;//Serial.write("Used, unknown meaning\n");
         
         if(x&0b100) data.alarms|=ALARM_TRANSDUCTER_DEFECTIVE;//Serial.write("Transducer defective\n");
         if(x&0b010) data.alarms|=ALARM_DEPTH;//Serial.write("Deep Alarm is active\n");
         if(x&0b001) data.alarms|=ALARM_SHALLOW_DEPTH;//Serial.write("Shallow Depth Alarm is active\n");
       }
     break;
     
     case 0x10:
        if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
           data.apparentwindangle=x/2.0;//print_float("Apparent Wind Angle:",((float)x)/2.0, LF);
        }
     break;
     
     case 0x11:
       if(serial.get_4(3)==1){
         float a=((float)(serial.get_8(2)&0x7f))+(((float)(serial.get_8(3)))/((float)10.0));
         data.apparentwindspeed=((serial.get_8(2)&0x80)?(kmh_to_knots(a/1000.0)):(a));

       }
     break;
     
     case 0x20:
       if(serial.get_4(3)==1){
         float a=((float)(serial.get_int(2,3)))/10;
         data.waterspeed=a;
       }
       break;
     
     case 0x21:
       if(serial.get_4(3)==1){
         float a=((float)( (serial.get_int(2,3)<<4) || (serial.get_int(4) )))/100.0;
         data.trip=a;
       }
       break;
       
    case 0x22:
       if(serial.get_4(3)==2){
         float a=((float)( serial.get_int(2,3)<<4) )/10.0;
         data.total=a;
       }
       break;
    
    case 0x23: //temperature
      if(serial.get_4(3)==2){
         if(serial.get_4(2)|0x4)
         {
           data.temp_sensor_error=true;
           ;//Serial.write("Water temperature sensor disconnected\n");
         }else
         {
           data.temp_sensor_error=false;
           data.watertemp=serial.get_8(2);
         }
       }
       break;
       
       
     case 0x24:
       if(serial.get_4(3)==2){
         uint8_t a=serial.get_8(4);
         if(!a) data.mil_sp_unit=UNIT_NM;//print_strs("Milage and speed dislpayed in",  NM,"/", KNOTS,LF);
         if(a==0x06) data.mil_sp_unit=UNIT_SM;//print_strs("Milage and speed dislpayed in ",SM,"/",MPH,LF);
         if(a==0x86) data.mil_sp_unit=UNIT_KM;//print_strs("Milage and speed dislpayed in ",KM,"/",KMH,LF);
       }
       break;
       
       
     case 0x25:
       if(serial.get_4(3)==4){
         uint8_t z=serial.get_4(2), x=serial.get_int(2),y=serial.get_int(3),u=serial.get_int(4),v=serial.get_int(5),
                   w=serial.get_4(13);
                   
         float total= ((float)(x+y*256+z* 4096))/ 10.0, trip=((float) (u+v*256+w*65536))/100.0;
         data.total=total;
         data.trip=trip;
        
       }
       break;
       
     case 0x26:
       if(serial.get_4(3)==4){
         uint16_t z=serial.get_4(2), x=serial.get_int(2,3),y=serial.get_int(4,5);
         uint8_t de=serial.get_8(6),kn=1, stopped=0;
         float current=((float)x)/100.0, average=((float)x)/100.0;
         
         if(de&0b1) stopped=1;
         if(de&0b10) kn=0;
         
         
         if(!stopped && ( de & 0b01000000))
         {
           data.speeds1=(de&2)?current:mi_to_nm(current);
         }
         
         if(!stopped)
         {
           data.speeds2=(de&2)?average:mi_to_nm(average);
         }
         
       }
       break;
       
      case 0x27:
       if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
         data.watertemp=((float)(x-100))/10.0;
       }
       break;
       
       case 0x30:
       if(serial.get_4(3)==0){
         uint8_t a=serial.get_8(2)&0x0f;
         if(!a) data.lamp_intensity=0;//Serial.write("LO\n");
         if(a==4) data.lamp_intensity=1;//Serial.write("L1\n");
         if(a==8) data.lamp_intensity=2;//Serial.write("L2\n");
         if(a=0xC) data.lamp_intensity=3;//Serial.write("L3\n");
       }
       break;
       
       case 0x36:
       if(serial.get_4(3)==0){
         data.mob_disable=false;
       }
       break;
       
       
       case 0x50: //latitude
       
       if(serial.get_4(3)==2){
         uint16_t x=serial.get_int(2), y=serial.get_int(3,4);
         float minutes=((float)(y&0x7fff ))/100.0;
         data.latitude=((float) x)+minutes/60.0;
       }
       break; 
       
       
       case 0x51: //longitude
       
       if(serial.get_4(3)==2){
         uint16_t x=serial.get_int(2), y=serial.get_int(3,4);
         float minutes=((float)(y&0x7fff ))/100.0;
         data.longitude=((float) x)+minutes/60.0;
       }
       break; 
       

       
       
       case 0x52:
       if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
         data.groundspeed=((float)x)/10.0;
       }
       break; 

       case 0x53:
       if(serial.get_4(3)==0){
         uint8_t u=serial.get_4(2), v=serial.get_8(2);
         data.magnetic_course=((float)(u & 0x3) * 90 + (v & 0x3F) * 2 + (u & 0xC)) / 8;
       }
       break;
       
       case 0x54:
       if(serial.get_4(3)==1){
         uint8_t t=serial.get_4(2), rs=serial.get_8(2), h=serial.get_8(3);
         uint16_t rst=(rs<<4)|t;
         data.time_h=h;
         data.time_m=((rs)&0xfc)>>2;
         data.time_s=((rst)&0x3f);
       }
       break;
       
       case 0x56:
       if(serial.get_4(3)==1){
         uint8_t m=serial.get_4(2), d=serial.get_8(2), y=serial.get_8(3);
         data.date_d=d;
         data.date_m=m;
         data.date_y=y;
       }
       break;
       
       
       case 0x57:
       if(serial.get_4(3)==1){
         data.n_sat=serial.get_4(2);
       }
       break;
       
       case 0x58:
       if(serial.get_4(3)==5){
         uint8_t z=serial.get_4(2), la=serial.get_8(2), x=serial.get_8(3),
                y=serial.get_8(4), lo=serial.get_8(5), q=serial.get_8(3),
                r=serial.get_8(4);
         float pos=(float)la+ (((float)x)*256.0+y)/1000.0;
         data.raw_latitude=((z & 1)?(-pos):(pos));
             
         pos=(float)lo+ (((float)q)*256.0+r)/1000.0;
         data.raw_latitude=((z & 2)?(-pos):(pos));
         
       }
       break;
       
       case 0x59:
       if(serial.get_4(3)==2 && serial.get_4(3)==0 && serial.get_4(3)==0xA){
         ;//Serial.write("It lefts 10 seconds (ST60 contdown)\n");
       }
       break;
       
       case 0x66:
       if(serial.get_4(3)==0){
         uint8_t x=serial.get_8(2),y;
         data.wind_alarms=x;
       }
       break;
       
       
       
       
       case 0x84:
       if(serial.get_4(3)==6){
         uint8_t u=serial.get_4(2), vw=serial.get_8(2), xy=serial.get_8(3),
                     z=serial.get_8(4), m=serial.get_8(5), r=serial.get_8(6),
                     s=serial.get_8(7),t=serial.get_8(8);
                     
          // compass heading
          data.compass_heading=(u&0x3)*90+(vw&0x3f)*2 + (u&0xc ? (u & 0xc==0xc ? 2 :1):0);
          ;//print_int("Compass heading: ",ch," degres\n");

          data.rudder_turning=u&0x8;
          
          //autopilot course
          data.autopilot_course=(vw>>6)*90+xy/2;
          
          data.autopilot_mode=z;
          
          //alarm
          if(m&0x4) data.alarms|=ALARM_OFF_COURSE;
          if(m&0x8) data.alarms|=ALARM_WIND_SHIFT;
          data.rudder_direction=r;
          
        
       }
       break;
       
     case 0x88:
       if(serial.get_4(3)==3){
         uint8_t w=serial.get_8(2),x=serial.get_8(3),
                 y=serial.get_8(4),z=serial.get_8(5);
         switch(w)
         {
           
           
           case 1: data.autopilot.rudder_gain=x; break;
           case 2: data.autopilot.counter_rudder=x;break;
           case 3: data.autopilot.rudder_limit=x;break;
           case 4: data.autopilot.turn_rate_speed=x;break;
           case 5: data.autopilot.speed=x;break;
           case 6: data.autopilot.off_course_limit=x;break;
           case 7: data.autopilot.trim=x;break;
           case 9: data.autopilot.power_steer=x;break;
           case 0XA: data.autopilot.drive_type=x;break;
           case 0xB: data.autopilot.rudder_damping=x;break;
           case 0xC: data.autopilot.variations=(int8_t)x;break;      
           case 0xD: data.autopilot.auto_adapt=x;break;
           case 0xE: data.autopilot.auto_adapt_latitude=x;break;
           case 0xF: data.autopilot.auto_release=x!=0;break;
           case 0x10: data.autopilot.rudder_alignement=x;break;
           case 0x11: data.autopilot.wind_trim=x;break;
           case 0x12: data.autopilot.response=x;break;
           case 0x13: data.autopilot.boat_type=x;break;
           case 0x15: data.autopilot.cal_lock=x!=0;break;
           case 0x1D: data.autopilot.tack_angle=x;break;
         }
       }
       break;
     
     case 0x89:
       if(serial.get_4(3)==2){
         uint8_t u=serial.get_4(2),vw=serial.get_8(2),
               xy=serial.get_8(3), z=serial.get_4(9);
          data.compass_heading=(u & 0x3) * 90 + (vw & 0x3F) * 2 +( (float)(u & 0xC)) / 2.0;
        
         data.stear_reference=(vw&0xc0)*90+((float)xy)/2;
         data.st40_mode=(z&2);
       }
     break;
     
     case 0x90:
       if(serial.get_4(3)==0){
         data.rudder_gain=serial.get_8(2); 
       }
       break;

     case 0x9C:
       if(serial.get_4(3)==1){
          uint8_t u=serial.get_4(2), vw=serial.get_8(2), r=serial.get_8(3);
         
          data.compass_heading=(u&0x3)*90+(vw&0x3f)*2 + (u&0xc ? (u & 0xc==0xc ? 2 :1):0);
          data.turning=u&0x8;
          
          //rudder (gouvernail)
          int8_t c=r, dir=(r&0x80);
          data.rudder_direction=r;
          
 
       }
       break;
     
     default:
       serial.write_frame();
       Serial.write("Datagramme inconnue\n");
       return -1;
       break;
   }
   t1=micros()-t1;
   if(nb)av=(t1+av*nb)/(++nb);
   else {av=t1;nb++;}
   print_float("Processing time: ", (float)t1 , "\n");
   print_float("Aaverage processing time: ", (float)av , "\n");
   delay(20); //pour mesures
   Serial.write("\n");
   return serial.get_cmd();
  
   
}
