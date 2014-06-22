/*
  decode_9bits.ino - SeaSerial library, to encapsulate Seatalk Datagrams
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
* Decode some SeaTalk Frames and print them
*
* For more information about SeaTalk protocol please visit:
* http://www.thomasknauf.de/seatalk.htm
* (by Thomas Knauf)
*/

#include <SeaSerial.h>
int i=0;

SeaSerial serial;


void setup()
{
        
	Serial.begin(115200);
        pinMode(13,OUTPUT);
        while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
        

        
}


void print_int(const char* bef, int x,const char* unity, const char* end=NULL)
{
  Serial.write(bef);
  Serial.print(x);
  Serial.write(unity);
  if(end) Serial.write(end);
}



void print_float(const char* bef, double x,const  char* unity, const char* end=NULL)
{
  Serial.write(bef);
  Serial.print(x,5);
  Serial.write(unity);
  if(end) Serial.write(end);
}

void print_bool(const char* bef, bool x,const  char* unity, const char* end=NULL)
{
   Serial.write(bef);
  if(x)Serial.print("true");
  else Serial.print("false");
  Serial.write(unity);
  if(end)Serial.write(end);
  
}

void print_if(bool cond,const  char *iftrue,const  char* iffalse, const char* end=NULL)
{
  
  Serial.write((cond)?iftrue:iffalse);
  if(end)Serial.write(end);
  
}


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

char buff[50];
void print_strs(const char* str1,const  char* str2=NULL, const char* str3=NULL, const char* str4=NULL, const char* str5=NULL)
{
  Serial.write(str1);
  if(str2) Serial.write(str2);
  if(str3) Serial.write(str3);
  if(str4) Serial.write(str4);
  if(str5) Serial.write(str5);

}

const char *LONGITUDE="Longitude ", *LATITUDE="Latitude ", *LEFT="left ", *RIGHT="right ",
    *SOUTH="south ", *NORTH="north ", *EAST="east ", *WEST="west ", *KNOTS="knots ",
    *LF="\n", *METPH="meter/second ",*MILES="miles ", *MPH="mph ", *DEG="degres ", *CEL="celsius ",*NM="nm ",
    *KM="km ", *KMH="km/h", *SM="sm ";

void loop()
{	
  
  //300/2500 octets de gagne
   
  unsigned long t1,t2;
  
  static double av=0.0;
  static int nb=0;
  
  t1=micros();
  serial.next_frame();
  
 
   serial.write_frame();
   print_int("Free ram: ",freeRam()," octets\n");
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
         print_float("Depth below transducter: ",x/10.0," feet",LF);
         if(y&0b1000) Serial.write("Anchor Alarm is active\n");
         if(y&0b0100) Serial.write("Metric display units \n");
         if(y&0b0010) Serial.write("Used, unknown meaning\n");
         
         if(x&0b100) Serial.write("Transducer defective\n");
         if(x&0b010) Serial.write("Deep Alarm is active\n");
         if(x&0b001) Serial.write("Shallow Depth Alarm is active\n");
       }
     break;
     
     case 0x10:
        if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
         print_float("Apparent Wind Angle:",((float)x)/2.0, LF);
        }
     break;
     
     case 0x11:
       if(serial.get_4(3)==1){
         float a=((float)(serial.get_8(2)&0x7f))+(((float)(serial.get_8(3)))/((float)10.0));
         print_float("Apparent Wind speed: ",
                       a,(serial.get_8(2)&0x80)?(METPH):(KNOTS),LF);
       }
     break;
     
     case 0x20:
       if(serial.get_4(3)==1){
         float a=((float)(serial.get_int(2,3)))/10;
         print_float("Speed throgh water: ",a,KNOTS,LF);
       }
       break;
     
     case 0x21:
       if(serial.get_4(3)==1){
         float a=((float)( (serial.get_int(2,3)<<4) || (serial.get_int(4) )))/100.0;
         print_float("Trip milage:",a,MILES,LF);
       }
       break;
       
    case 0x22:
       if(serial.get_4(3)==2){
         float a=((float)( serial.get_int(2,3)<<4) )/10.0;
         print_float("Total milage:",a,MILES,LF);
       }
       break;
    
    case 0x23: //temperature
      if(serial.get_4(3)==2){
         if(serial.get_4(2)|0x4)
         {
           Serial.write("Water temperature sensor disconnected\n");
         }else
         {
           print_int("Water temperature: ",serial.get_8(2), DEG, CEL);
         }
       }
       break;
       
       
     case 0x24:
       if(serial.get_4(3)==2){
         uint8_t a=serial.get_8(4);
         if(!a) print_strs("Milage and speed dislpayed in",  NM,"/", KNOTS,LF);
         if(a==0x06) print_strs("Milage and speed dislpayed in ",SM,"/",MPH,LF);
         if(a==0x86) print_strs("Milage and speed dislpayed in ",KM,"/",KMH,LF);
       }
       break;
       
       
     case 0x25:
       if(serial.get_4(3)==4){
         uint8_t z=serial.get_4(2), x=serial.get_int(2),y=serial.get_int(3),u=serial.get_int(4),v=serial.get_int(5),
                   w=serial.get_4(13);
                   
         float total= ((float)(x+y*256+z* 4096))/ 10.0, trip=((float) (u+v*256+w*65536))/100.0;
         print_float("Total: ",total,MILES,LF);
         print_float("Trip: ",trip,MILES,LF);
        
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
           print_float("Speed from sensor 1:",current,(kn)?(KNOTS):(MPH),LF);
         }
         
         if(!stopped)
         {
           print_float( (de & 0b10000000,"Speed from sensor 2:",
                       "Average trip speed:"),
                       average,
                       (kn)?(" knots\n"):(" mph\n"));
         }
         
       }
       break;
       
      case 0x27:
       if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
         float temp=((float)(x-100))/10.0;
         print_float("water temprature: ",temp, CEL,LF);
       }
       break;
       
       case 0x30:
       if(serial.get_4(3)==0){
         uint8_t a=serial.get_8(2)&0x0f;
         Serial.write("Lamp intensity: ");
         if(!a) Serial.write("LO\n");
         if(a==4) Serial.write("L1\n");
         if(a==8) Serial.write("L2\n");
         if(a=0xC) Serial.write("L3\n");
       }
       break;
       
       case 0x36:
       if(serial.get_4(3)==0){
         Serial.write("MOB (man over board) disabled\n");
       }
       break;
       
       
       case 0x38:
       if(serial.get_4(3)==1){
         Serial.write("Codelock data...\n");
       }
       
       case 0x50: //latitude
       
       if(serial.get_4(3)==2){
         uint16_t x=serial.get_int(2), y=serial.get_int(3,4);
         float minutes=((float)(y&0x7fff ))/100.0;
         float deg=((float) x)+minutes/60.0;
         print_float(LATITUDE,deg,
             ((0x8000 & y)?(SOUTH):(NORTH)), LF );
       }
       break; 
       
       
       case 0x51: //longitude
       
       if(serial.get_4(3)==2){
         uint16_t x=serial.get_int(2), y=serial.get_int(3,4);
         float minutes=((float)(y&0x7fff ))/100.0;
         float deg=((float) x)+minutes/60.0;
         print_float(LONGITUDE,deg,
             ((0x8000 & y)?(EAST):(WEST)),LF );
       }
       break; 
       

       
       
       case 0x52:
       if(serial.get_4(3)==1){
         uint16_t x=serial.get_int(2,3);
         float sog=((float)x)/10.0;
         print_float("Speed over ground: ",sog,KNOTS,LF);
       }
       break; 

       case 0x53:
       if(serial.get_4(3)==0){
         uint8_t u=serial.get_4(2), v=serial.get_8(2);
         float mag=((float)(u & 0x3) * 90 + (v & 0x3F) * 2 + (u & 0xC)) / 8;
         print_float("Magnetic course: ",mag,DEG,LF);
       }
       break;
       
       case 0x54:
       if(serial.get_4(3)==1){
         uint8_t t=serial.get_4(2), rs=serial.get_8(2), h=serial.get_8(3);
         uint16_t rst=(rs<<4)|t;
         Serial.write("GMT Time:");
         Serial.print(h);
         Serial.write(" : ");
         Serial.print( ((rs)&0xfc)>>2 );
         Serial.write(" : ");
         Serial.print( ((rst)&0x3f) );
         Serial.write("\n");
       }else Serial.write("GMT = NULL\n");
       break;
       
       case 0x56:
       if(serial.get_4(3)==1){
         uint8_t m=serial.get_4(2), d=serial.get_8(2), y=serial.get_8(3);
         Serial.write("Date (dd/mm/yy):");
         Serial.print(d);
         Serial.write("/");
         Serial.print(m);
         Serial.write("/");
         Serial.print(y);
         Serial.write("\n");
       }
       break;
       
       
       case 0x57:
       if(serial.get_4(3)==1){
         uint8_t m=serial.get_4(2), d=serial.get_8(2), y=serial.get_8(3);
         Serial.write("Date (dd/mm/yy):");
         Serial.print(d);
         Serial.write("/");
         Serial.print(m);
         Serial.write("/");
         Serial.print(y);
         Serial.write("\n");
       }
       break;
       
       case 0x58:
       if(serial.get_4(3)==5){
         uint8_t z=serial.get_4(2), la=serial.get_8(2), x=serial.get_8(3),
                y=serial.get_8(4), lo=serial.get_8(5), q=serial.get_8(3),
                r=serial.get_8(4);
         float pos=(float)la+ (((float)x)*256.0+y)/1000.0;
         print_float(LATITUDE,pos,
             ((z & 1)?(SOUTH):(NORTH)),LF );
             
         pos=(float)lo+ (((float)q)*256.0+r)/1000.0;
         print_float(LONGITUDE,lo,
             ((z&2)?(EAST):(WEST)),LF );
         
       }
       break;
       
       case 0x59:
       if(serial.get_4(3)==2 && serial.get_4(3)==0 && serial.get_4(3)==0xA){
         Serial.write("It lefts 10 seconds (ST60 contdown)\n");
       }
       break;
       
       case 0x66:
       if(serial.get_4(3)==0){
         uint8_t x=serial.get_8(2),y;
         y=x>>4;
         char ap[]="Apparent ", tr[]="True ", wi[]="Wind ",an[]="angle ", sp[]="speed ",
                       low[]="low\n",high[]="high\n";
         if(x) Serial.write("Alarm: ");
         if(x&8)print_strs(ap,wi,an,low);
         if(x&4)print_strs(ap,wi,an,high);
         if(x&2)print_strs(ap,wi,sp,low);
         if(x&1)print_strs(ap,wi,sp,high);
         if(y&8)print_strs(tr,wi,an,low);
         if(y&4)print_strs(tr,wi,an,high);
         if(y&2)print_strs(tr,wi,sp,low);
         if(y&1)print_strs(tr,wi,sp,high);
         if(!x) Serial.write("End of alarm");
       }
       break;
       
       
       
       
       case 0x84:
       if(serial.get_4(3)==6){
         uint8_t u=serial.get_4(2), vw=serial.get_8(2), xy=serial.get_8(3),
                     z=serial.get_8(4), m=serial.get_8(5), r=serial.get_8(6),
                     s=serial.get_8(7),t=serial.get_8(8);
                     
          // compass heading
          uint16_t ch=(u&0x3)*90+(vw&0x3f)*2 + (u&0xc ? (u & 0xc==0xc ? 2 :1):0);
          print_int("Compass heading: ",ch," degres\n");
          
          //turning direction
          Serial.write("Turning ");
          print_if(u&0x8, RIGHT, LEFT, LF);
          
          //autopilot course
          uint16_t autop=(vw>>6)*90+xy/2;
          print_int("Autopilot course: ",autop,DEG,LF);
          
          //mode
          print_if(z&2, "Autopilot in auto mode\n","Autopilot in stand-by mode\n");
          if(z&4) Serial.write("Autopilot in Vane Mode\n");
          if(z&8) Serial.write("Autopilot in Track Mode\n");
          
          //alarm
          if(m&0x4) Serial.write("Alarms off course\n");
          if(m&0x8) Serial.write("Alarms wind shift\n");
          
          //rudder (gouvernail)
          int8_t c=r, dir=(r&0x80);
          if(c<0) c=-c;
          print_int("Rudder direction: ",c, DEG,
                ((dir)?(LEFT):(RIGHT)) );
          Serial.write(LF);
          
        
       }
       break;
       
     case 0x85:
       if(serial.get_4(3)==6){
         uint16_t zz=serial.get_8(5)| (serial.get_4(8)<<8);
         uint8_t y=serial.get_4(12);
         uint16_t crosstrack=serial.get_8(2);
         crosstrack|=serial.get_4(2)<<8;
         print_float("Cross track error:",(float) ((float)crosstrack)/100.0, NM, LF);
         
         //float bearing_to_dest=90*serial.get_4(7) + 
         //          (serial.get_4(6) || (serial.get_4(9)<<4))/2.0;
         print_float("Bearing to destination:", 23.3,DEG);
         
         if(serial.get_4(9)&4) Serial.write("Bearing is true\n");
         else Serial.write("Bearing is magnetic\n");
        
         print_float("Distance to destination: ",
                               ((y&1)?((float) (zz/*100.0*/)):((float) (zz/10.0))),
                               NM, LF);
         
         print_strs("Steer ",(y&4)?(RIGHT):(LEFT),"to correct error",LF);
       }
       break;
       
     case 0x88:
       if(serial.get_4(3)==3){
         uint8_t w=serial.get_8(2),x=serial.get_8(3),
                 y=serial.get_8(4),z=serial.get_8(5);
         Serial.write("Auto pilote parameter:\n");
         switch(w)
         {
           case 1: print_int("Rudder gain:",x,"/9\n");break;
           case 2: print_int("counter rudder:",x,"/9\n");break;
           case 3: print_int("rudder limit:",x,"/40\n");break;
           case 4: print_int("turn rate limit:",x,"/30\n");break;
           case 5: print_int("speed:",x,"/60\n");break;
           case 6: print_int("off course limit:",x,"/40\n");break;
           case 7: print_int("auto trim :",x,"/4");break;
           case 9: print_bool("power steer (joystick):",x!=0,"\n");break;
           case 0XA: print_int("drive type:",x,"/ (3,4 or 5)\n");break;
           case 0xB: print_int("rudder damping :",x,"/9\n");break;
           case 0xC: print_int("variation:",(int8_t)x,"/(-30/30)\n");break;
           case 0xD: 
               Serial.write("auto adapt:");
               if(!x) Serial.write("off\n");
               else if(x==1) print_strs(NORTH,LF);
               else if(x==2) print_strs(SOUTH,LF);
           break;
           case 0xE: print_int("auto adapt latitude:",x,"/80\n");break;
           case 0xF: print_bool("auto release:",x,"\n");break;
           case 0x10: print_int("rudder alignment:",(uint8_t)x,"/(-7/7)\n");break;
           case 0x11: print_int("Wind Trim (Wind Response):",x,"/9\n");break;
           case 0x12: print_int("Response:",x,"/9\n");break;
           case 0x13: Serial.write("Boat type: ");
              if(x==1) Serial.write("displ\n");
              if(x==2) Serial.write("semi-displ\n");
              if(x==3) Serial.write("plan\n");
              if(x==4) Serial.write("Stern\n");
              if(x==5) Serial.write("work\n");
              if(x==6) Serial.write("sail\n");
           break;
           case 0x15: print_bool("Cal lock:",!x,"\n");break;
           case 0x1D: print_int("Auto Tack Angle:",x,"/(40-125)\n");break;
         }
       }
       break;
     
     case 0x89:
       if(serial.get_4(3)==2){
         uint8_t u=serial.get_4(2),vw=serial.get_8(2),
               xy=serial.get_8(3), z=serial.get_4(9);
         print_float("compass heading direction: ",
             (u & 0x3) * 90 + (vw & 0x3F) * 2 +( (float)(u & 0xC)) / 2.0,
             DEG,LF);
         print_float("Locked stear reference: ", (vw&0xc0)*90+((float)xy)/2,
                    DEG,LF);
         if(z&2) Serial.write("St40 in Standby mode \n");
         else Serial.write("St40 in Locked stear mode  \n");
       }
     break;
     
     case 0x90:
       if(serial.get_4(3)==0){
         uint8_t gain=serial.get_8(2);
         print_int("Rudder gain to: ",gain,LF);
 
       }
       break;

     case 0x9C:
       if(serial.get_4(3)==1){
          uint8_t u=serial.get_4(2), vw=serial.get_8(2), r=serial.get_8(3);
         
          uint16_t ch=(u&0x3)*90+(vw&0x3f)*2 + (u&0xc ? (u & 0xc==0xc ? 2 :1):0);
          print_int("Compass heading: ",ch,DEG,LF);
          
          //turning direction
          Serial.write("turinng ");
          print_if(u&0x8, RIGHT, LEFT, LF);
          
          //rudder (gouvernail)
          int8_t c=r, dir=(r&0x80);
          if(c<0) c=-c;
          print_int("Rudder direction: ",c, DEG, (dir)?(LEFT):(RIGHT));
          Serial.print("\n");
 
       }
       break;
     
     default:
       
       Serial.write("Datagramme inconnue\n");
       break;
   }
   t1=micros()-t1;
   if(nb)av=(t1+av*nb)/(++nb);
   else {av=t1;nb++;}
   print_float("Processing time: ", (float)t1 , "\n");
   print_float("Aaverage processing time: ", (float)av , "\n");
   //delay(20); //pour mesures
   Serial.write("\n");
  
   
}
