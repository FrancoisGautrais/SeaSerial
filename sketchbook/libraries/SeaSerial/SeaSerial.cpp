/*
  HardwareSerial.cpp - SeaSerial library, to encapsulate Seatalk Datagrams
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

#include "SeaSerial.h"

SeaSerial::SeaSerial(HardwareSerial& s,uint16_t bauds, uint16_t flag) : serial(s)
{
  size=0;
  serial.begin(4800,SERIAL_9N1);
}


/*
* Read the next frame from serial
*/
size_t SeaSerial::next_frame()
{
    size=19; //max size + 1
    int i=0;
   while(i!=(size))
    {
        if(serial.available())
        {
          uint16_t c=serial.read(); //Next 9 bit charater
          if(i==0 &&  (c&0x100 )==0) continue; //Beginning of the datagram
          else if(i==1)    size=(c&0x0f)+3; //Size is the lowest part of second character
          frame[i]=(uint8_t)c;
          i++;
        }
    }
    return size;
}

/*
*
*/
void SeaSerial::write_frame()
{
  int i;
  char tmp[5];
  Serial.write("Frame: ");
  for(i=0; i<size; i++)
  {
     sprintf(tmp, "%x ",frame[i]);
     Serial.write(tmp);
  }
  Serial.write("\n");
}

/*
* Get the size of datagram
*/
size_t SeaSerial::get_size()
{
   return size;
}

/*
* Get the raw datagram (8 bits characters)
*/
char* SeaSerial::get_frame()
{
  return (char*)frame;
}
