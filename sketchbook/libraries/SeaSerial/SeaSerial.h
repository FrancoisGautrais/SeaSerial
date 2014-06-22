/*
  HardwareSerial.h - SeaSerial library, to encapsulate Seatalk Datagrams
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

#ifndef SEA_SERIAL_H
#define SEA_SERIAL_H

#include <HardwareSerial.h>

class SeaSerial
{
  private:
    uint8_t frame[18];
    size_t size;
    HardwareSerial& serial;


  public:
    SeaSerial(HardwareSerial& =Serial1,uint16_t bauds=4800, uint16_t=SERIAL_9N1);
    size_t next_frame();
    size_t get_size();
    char* get_frame();
    void write_frame();
    uint8_t get_cmd() { return frame[0];}
    uint8_t get_8(int i){ return frame[i];}
    uint8_t get_4(int i)
    {
        if(i%2==0) return (frame[i/2]>>4)&0xf;
        else return (frame[i/2])&0x0f;
    }



    uint32_t get_int(int i1, int i2=-1, int i3=-1, int i4=-1)
    {
      uint32_t x=frame[i1];
      if(i2>=0) x|=frame[i2]<<8;
      if(i3>=0) x|=frame[i3]<<16;
      if(i4>=0) x|=frame[i4]<<24;
      return x;
    }
};

#endif // SEA_SERIAL_H
