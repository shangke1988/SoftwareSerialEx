/*
SoftwareSerialEx.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

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

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerialEx.h>
#include <util/delay_basic.h>

#if _DEBUG
#include "HardwareSerial.h"
#endif

/******************************************************************************
* HardwareTimer
******************************************************************************/

HardwareTimer::HardwareTimer(volatile uint8_t *tccra, volatile uint8_t *tccrb, volatile uint16_t *tcnt, volatile uint16_t *ocr, volatile uint8_t *timsk, uint8_t ocie, volatile uint8_t *tifr, uint8_t ocf):
  _tccra(tccra),
  _tccrb(tccrb),
  _tcnt(tcnt),
  _ocr(ocr),
  _timsk(timsk),
  _bv_ocie(_BV(ocie)),
  _tifr(tifr),
  _bv_ocf(_BV(ocf))
{
  *tccra = 0x00;
  *tccrb = 0x01;
  //*timsk = _BV(OCIE1B);
  count = 0;
  for(uint8_t i=0;i<_TIMER_MAX_;i++)
  {
    used[i]=0;
	heap[i]=0;
  }  
}
// This function add a timer to the timer heap
int8_t HardwareTimer::addtimer(uint16_t ocr, uint16_t add, tcfunc func, void *target)
{
  if(count<_TIMER_MAX_)
  {
    uint16_t tcnt;
    uint8_t k;
	uint8_t oldSREG = SREG;
	cli();
	tcnt = *_tcnt;
	if(count>0)
    {
	  if(*_tifr & _bv_ocf)
	  {
	    *_tifr = _bv_ocf;
	    timer_comp_irq();
	  }
    }
/*	while((*_tcnt-tcnt) +16 > (ocr-tcnt))
	{
	  uint8_t rec=(*func)(target);
	  if(rec==0)
	  {
	    return _TIMER_MAX_;
	  }
	  tcnt = ocr;
	  ocr += add;	  
	}
	tcnt = *_tcnt;*/
	for(k=0;k<_TIMER_MAX_-1;k++)
	{
	  if(used[k]==0)
	    break;
	}
    list[k].ocr = ocr;
	list[k].add = add;
	list[k].func = func;
	list[k].target = target;
	used[k]=1;
	
	if(count==0)
	{
	  *_ocr=ocr;
	  heap[0] = k;
	  *_tifr = _bv_ocf;
	  *_timsk |= _bv_ocie;	  
	  count++;
	}else
	{
	  heap[count] = k;
	  // heap sort
	  uint8_t t1,t2;
	  t1 = count;
	  count++;
	  while(t1>0)
	  {
        t2 = (t1-1)/2;
		if((list[heap[t2]].ocr - tcnt) > (list[k].ocr - tcnt))
		{
		  heap[t1] = heap[t2];
		  heap[t2] = k;
		  t1=t2;
		}else
		{
		  break;
		}
	  }
	  if(t1==0)
	  {
	    *_ocr=ocr;
		if((*_tcnt-tcnt) > (ocr-tcnt))
	    {
	      *_tifr = _bv_ocf;
	      timer_comp_irq();
	    }	
	  }	  
	}	
	SREG = oldSREG;	
    return k;
  }else
  {
    return -1;
  }
}
// This function delete a timer from the timer heap
void HardwareTimer::deltimer(int8_t k)
{
	uint16_t tcnt=*_tcnt;
	uint8_t oldSREG = SREG;
	cli();
	if(used[k]==0)
	{
		return;
	}
	used[k]=0;
	uint8_t t;
	for(t=0;t<count;t++)
	{
		if(heap[t]==k)
			break;
	}
	if(t==count)
	{
		#if _DEBUG
		Serial.println(F("ERROR:deltimer:can't find k in heap."));
		Serial.print(F("count="));
		Serial.print(count);
		Serial.print(F(",k="));
		Serial.print(k);
		Serial.print(F(",add="));
		Serial.println(list[k].add);
		#endif
		return;
	}
	bool isocr = (t==0);
	count--;
	if(count==0)
	{	
		*_timsk &= ~_bv_ocie;
		return;
	}
	k = heap[count];
	//resort heap	
	uint16_t ocr_t = list[k].ocr-tcnt;
	uint8_t t1,t2;
	for(;;)
	{
		t1 = 2*t+1;
		t2 = t1+1;
		if(t2<count)
		{
			uint16_t ocr_t1=list[heap[t1]].ocr-tcnt;
			uint16_t ocr_t2=list[heap[t2]].ocr-tcnt;
			if(ocr_t1 < ocr_t2)
			{
				if(ocr_t<ocr_t1)
				{
					heap[t] = k;
					break;
				}else
				{
					heap[t] = heap[t1];
					t = t1;
				}
			}else
			{
				if(ocr_t<ocr_t2)
				{
					heap[t] = k;
					break;
				}else
				{
					heap[t] = heap[t2];
					t = t2;
				}
			}
		}else if(t1<count)
		{
			uint16_t ocr_t1=list[heap[t1]].ocr-tcnt;
			if(ocr_t<ocr_t1)
			{
				heap[t] = k;
				break;
			}else
			{
				heap[t] = heap[t1];
				heap[t1] = k;
				break;
			}
		}else
		{
			heap[t] = k;
			break;
		}
	}//for
	
	//set ocr
	if(isocr)
	{
		k = heap[0];
		*_ocr = list[k].ocr;
		if((*_tcnt-tcnt) > (*_ocr-tcnt))
	    {
	      *_tifr = _bv_ocf;
	      timer_comp_irq();
	    }	
	}
	
	SREG = oldSREG;	
}
// This function is the interrupt call function
void HardwareTimer::timer_comp_irq()
{
	uint16_t tcnt;
	do
	{
		tcnt=*_ocr;
		*_tifr = _bv_ocf;//in case ocf set when do-while last several commands, perhaps impossible
		uint8_t k = heap[0];
	#if _DEBUG
		if(used[k]==0)
		{
			Serial.println(F("ERROR:time_irq:used[k]==0"));
			Serial.print(F("count="));
			Serial.print(count);
			Serial.print(F(",k="));
			Serial.print(k);
			Serial.print(F(",add="));
			Serial.println(list[k].add);
			return;
		}
	#endif
		tcfunc func = list[k].func;
		void* target = list[k].target;
		uint8_t rec = (*func)(target);
		if(rec==0)
		{
			//delete
			#if _DEBUG
			if(count==0)
			{
				Serial.println(F("ERROR:time_irq:delete:count==0"));
				count = 1;
			}
			#endif
			used[k] = 0;
			count--;
			if(count==0)
			{
				*_timsk &= ~_bv_ocie;
				return;
			}else
			{
				k = heap[count]; 
			}
		}else
		{
			//add ocr
			list[k].ocr += list[k].add;
		}
		//resort heap	
		uint16_t ocr_t = list[k].ocr-tcnt;
		uint8_t t=0,t1,t2;
		for(;;)
		{
			t1 = 2*t+1;
			t2 = t1+1;
			if(t2<count)
			{
				uint16_t ocr_t1=list[heap[t1]].ocr-tcnt;
				uint16_t ocr_t2=list[heap[t2]].ocr-tcnt;
				if(ocr_t1 < ocr_t2)
				{
					if(ocr_t<ocr_t1)
					{
						heap[t] = k;
						break;
					}else
					{
						heap[t] = heap[t1];
						t = t1;
					}
				}else
				{
					if(ocr_t<ocr_t2)
					{
						heap[t] = k;
						break;
					}else
					{
						heap[t] = heap[t2];
						t = t2;
					}
				}
			}else if(t1<count)
			{
				uint16_t ocr_t1=list[heap[t1]].ocr-tcnt;
				if(ocr_t<ocr_t1)
				{
					heap[t] = k;
					break;
				}else
				{
					heap[t] = heap[t1];
					heap[t1] = k;
					break;
				}
			}else
			{
				heap[t] = k;
				break;
			}
		}//for
		//set ocr
		k = heap[0];
		*_ocr = list[k].ocr;
	}while((*_tcnt-tcnt)>(*_ocr-tcnt));
}



HardwareTimer Timer(&TCCR1A,&TCCR1B,&TCNT1,&OCR1B,&TIMSK1,OCIE1B,&TIFR1,OCF1B);

ISR(TIMER1_COMPB_vect)
{
  Timer.timer_comp_irq();
}


/******************************************************************************
* SoftwareSerialEx
******************************************************************************/

