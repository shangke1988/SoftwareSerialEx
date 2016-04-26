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
#define _DEBUG 1
//#define _DEBUG_PIN1 11
//#define _DEBUG_PIN2 13
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

HardwareTimer Timer(&TCCR1A,&TCCR1B,&TCNT1,&OCR1B,&TIMSK1,OCIE1B,&TIFR1,OCF1B);

ISR(TIMER1_COMPB_vect)
{
  Timer.timer_comp_irq();
}
// construction function
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
// Becasue the wiring.c will use time1 as PWM, we need init in setup() again.
void HardwareTimer::init()
{
	*_tccra = 0x00;
	*_tccrb = 0x01;
	//*_tccrb = 0x05;
}
// This function add a timer to the timer heap
int8_t HardwareTimer::addtimer(uint16_t ocr, uint16_t add, timer_func func, void *target)
{
  if(count<_TIMER_MAX_ && func)
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
		timer_func func = list[k].func;
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

/******************************************************************************
* PCINTController
******************************************************************************/
#ifdef PCIE0
	PCINTController PCINT_Ctrl0(&PCICR,PCIE0,&PCMSK0,&PINB);
	ISR(PCINT0_vect)
	{
		PCINT_Ctrl0.pcint_irq();
	}
	#ifdef PCIE1
		PCINTController PCINT_Ctrl1(&PCICR,PCIE1,&PCMSK1,&PINC);
		ISR(PCINT1_vect)
		{
			PCINT_Ctrl1.pcint_irq();
		}
		#ifdef PCIE2
			PCINTController PCINT_Ctrl2(&PCICR,PCIE2,&PCMSK2,&PIND);
			ISR(PCINT2_vect)
			{
				PCINT_Ctrl2.pcint_irq();
			}
			#define PCI_CTRL(n)  (((n)==0) ? (&PCINT_Ctrl0) : (((n)==1) ? (&PCINT_Ctrl1) : (&PCINT_Ctrl2)))
		#else
			#define PCI_CTRL(n)  (((n)==0) ? (&PCINT_Ctrl0) : (&PCINT_Ctrl1))
		#endif
	#else
		#define PCI_CTRL(n)  (&PCINT_Ctrl0)
	#endif
#endif


PCINTController::PCINTController(volatile uint8_t *pcicr, uint8_t pcie, volatile uint8_t *pcmsk, volatile uint8_t *pin):
//  _pcicr(pcicr),
//  _bv_pcie(_BV(pcie)),
  _pcmsk(pcmsk),
  _pin(pin)
{
	//*pcmsk = 0;
	*pcicr |= _BV(pcie);
	for(uint8_t i=0;i<8;i++)
	{
		func_list[i] = NULL;
	}
}

void PCINTController::enable(uint8_t bv)
{
	//uint8_t bv = _BV(bit);
	uint8_t oldSREG = SREG;
	cli();
	*_pcmsk |= bv;
	SREG = oldSREG;
}
void PCINTController::disenable(uint8_t bv)
{
	//uint8_t bv = _BV(bit);
	uint8_t oldSREG = SREG;
	cli();
	*_pcmsk &= ~bv;
	SREG = oldSREG;
}

void PCINTController::addpcint(uint8_t bit, SoftwareSerialEx* ss)
{
	if(bit<8 && ss!=NULL)
	{
		uint8_t bv = _BV(bit);
		disenable(bv);
		func_list[bit] = ss;
		uint8_t i;
		for(i=0;i<count;i++)
		{
			if(mask_list[i].bit == bit)
				break;
		}
		if(i<8)
		{
			mask_list[i].mask = bv;
			mask_list[i].bit = bit;
			if(i==count)
			{
				count++;
			}
			enable(bv);
		}
		#if _DEBUG
		else
			Serial.println(F("ERROR:addpcint:unreachable."));
		#endif		
	}
}

void PCINTController::delpcint(uint8_t bit)
{
	if(bit<8)
	{
		disenable(_BV(bit));
		func_list[bit] = NULL;
		for(uint8_t i=0;i<count;i++)
		{
			if(mask_list[i].bit == bit)
			{
				uint8_t oldSREG = SREG;
				cli();
				for(;i<count-1;i++)
				{
					mask_list[i]=mask_list[i+1];
				}
				count--;
				SREG = oldSREG;
			}
		}		
	}
}

void PCINTController::pcint_irq()
{
	uint16_t tcnt = Timer.gettcnt();
	uint8_t pin = *_pin;
	pin = (~pin) & (*_pcmsk);
	if(pin)
	{
		for(uint8_t i=0;i<count;i++)
		{
			if(pin & mask_list[i].mask)
			{
				if(func_list[i])
				{
					uint8_t rec = func_list[i]->recv_start(tcnt);
					if(!rec)
					{
						*_pcmsk &= ~mask_list[i].mask;
					}					
				}
			}
		}
	}
}
/******************************************************************************
* SoftwareSerialEx
******************************************************************************/
SoftwareSerialEx::SoftwareSerialEx(uint8_t rx, uint8_t tx)
{
	//initialise variables
	_listen=0;
	//initialise buffers
	_rx_buffer_tail = 0;
	_rx_buffer_head = 0;
	_tx_buffer_tail = 0;
	_tx_buffer_head = 0;
	_tx_buffer_empty = 1;
	_tx_error = 0;
	//initialise delay
	_rx_delay_centering = 0;
	_bit_delay  = 0;
	
	//set tx pin
	digitalWrite(tx, HIGH);
	pinMode(tx, OUTPUT);
	_tx_BitMask = digitalPinToBitMask(tx);
	_tx_BitMask_inv = ~_tx_BitMask;
	uint8_t port = digitalPinToPort(tx);
	_tx_Port = portOutputRegister(port);
	//set rx pin
	pinMode(rx, INPUT);
	digitalWrite(rx, HIGH); 
	_rx_Pin = rx;
	_rx_BitMask = digitalPinToBitMask(rx);
	port = digitalPinToPort(rx);
	_rx_Port = portInputRegister(port);
	//set pcint
	_ispcint = (digitalPinToPCICR(_rx_Pin)!=0);
	if(_ispcint)
	{
		uint8_t pcin=digitalPinToPCICRbit(_rx_Pin);
		_pci_ctrl = PCI_CTRL(pcin);
		_pcint_bit = digitalPinToPCMSKbit(_rx_Pin);
		_pcint_maskvalue = _BV(_pcint_bit);	
	}else
	{
		_pci_ctrl = NULL;
		_pcint_bit = 0;
		_pcint_maskvalue = 0;
	}
}
SoftwareSerialEx::~SoftwareSerialEx()
{
	stoplisten();
}
void SoftwareSerialEx::listen()
{
	if(_ispcint && (!_listen))
	{
		_pci_ctrl->addpcint(_pcint_bit, this);
		_listen = 1;
	}
}
void SoftwareSerialEx::stoplisten()
{
	if(_ispcint && _listen)
	{
		_pci_ctrl->delpcint(_pcint_bit);
		_listen = 0;
	}
}
void SoftwareSerialEx::enable_pcint()
{
	// consider the speed, hasn't checked _ispcint.
	_pci_ctrl->enable(_pcint_maskvalue);
}
void SoftwareSerialEx::disenable_pcint()
{
	// consider the speed, hasn't checked _ispcint.
	_pci_ctrl->enable(_pcint_maskvalue);
}
void SoftwareSerialEx::begin(long speed)
{
	_bit_delay = (F_CPU + speed/2)/ speed;
	if(_ispcint)
	{
		//round, not floor. perhaps unuseful, but no bad.		
		_rx_delay_centering = (F_CPU + speed)/ (speed*2);		
		listen();
	}	
}
uint8_t SoftwareSerialEx::recv_start(uint16_t tcnt)
{
	uint8_t rec = Timer.addtimer(tcnt+_rx_delay_centering, _bit_delay, (timer_func)rx_irq, (void*)this);
	if(rec<0)
	{
		#if _DEBUG
		Serial.println(F("ERROR:recv_start:can't addtimer."));
		#endif
		//enable pcint
		return 1;
	}
	_rx_bitnum = 0;
	_rx_data = 0;
	//disenable pcint
	return 0;
}
uint8_t SoftwareSerialEx::rx_irq(SoftwareSerialEx* p)
{
	return p->recv_bits();
}
uint8_t SoftwareSerialEx::tx_irq(SoftwareSerialEx* p)
{
	return p->transmit();
}
#define RX_PIN (*_rx_Port & _rx_BitMask)
uint8_t SoftwareSerialEx::recv_bits()
{
	_rx_bitnum++;
	//start bit
	if(_rx_bitnum == 1)
	{
		if(RX_PIN == 0)
		{
			return 1;
		}else
		{
			enable_pcint();
			return 0;
		}
	}
	//data bits
	_rx_data >>= 1;
	if(RX_PIN)
		_rx_data |= 0x80;
	if(_rx_bitnum == 9)
	{
		//store buffer
		uint8_t next = (_rx_buffer_tail + 1) % _SS_RX_BUFF_SIZE;
		if(next!=_rx_buffer_head)
		{
			_rx_buffer[_rx_buffer_tail] = _rx_data;
			_rx_buffer_tail = next;
		}else
		{
			_rx_buffer_overflow = 1;
		}
		enable_pcint();
		return 0;
	}
	// Ignore stop bit. Pcint process the raise edge of 8th bit is fast than timer.
	// Because the timer with sorting is very slow, pcint check pins low before scan table. 
	// If pcint only process fall edge, it is perfect. But there is only two int pins in ATmega328.
	return 1;
}
uint8_t SoftwareSerialEx::transmit()
{
	_tx_bitnum++;
	// start bit
	if(_tx_bitnum == 1)
	{
		*_tx_Port &= _tx_BitMask_inv;
		return 1;
	}
	// data bits
	if(_tx_bitnum <= 9)
	{
		if(_tx_data&0x01)
			*_tx_Port |= _tx_BitMask;
		else
			*_tx_Port &= _tx_BitMask_inv;
		_tx_data >>= 1;
		return 1;
	}
	// stop bit
	*_tx_Port |= _tx_BitMask;
	if(_tx_bitnum >= 9 + _SS_STOP_BIT)
	{
		_tx_bitnum = 0;
		if(_tx_buffer_tail == _tx_buffer_head)
		{
			_tx_buffer_empty = 1;
			return 0;
		}else
		{
			_tx_data = _tx_buffer[_tx_buffer_head];
			_tx_buffer_head = (_tx_buffer_head + 1) % _SS_TX_BUFF_SIZE;
			return 1;
		}		
	}
	return 1;
}
int SoftwareSerialEx::available(void)
{
  	uint8_t head = _rx_buffer_head;
	uint8_t tail = _rx_buffer_tail;
	if (head > tail) return _SS_RX_BUFF_SIZE - head + tail;
	return tail - head;
}
int SoftwareSerialEx::availableForWrite(void)
{
	uint8_t head = _tx_buffer_head;
	uint8_t tail = _tx_buffer_tail;
	if (head >= tail) return _SS_TX_BUFF_SIZE - 1 - tail + head;
	return head - tail - 1;
}
int SoftwareSerialEx::peek(void)
{
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    return _rx_buffer[_rx_buffer_head];
  }
}
int SoftwareSerialEx::read(void)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer_head == _rx_buffer_tail) {
		return -1;
	} else {
		unsigned char c = _rx_buffer[_rx_buffer_head];
		_rx_buffer_head = (_rx_buffer_head + 1) % _SS_RX_BUFF_SIZE;
		return c;
	}
}
size_t SoftwareSerialEx::write(uint8_t byte)
{
	if(!_bit_delay)
		return 0;
	
	if(_tx_buffer_empty)
	{
		if(!_tx_error)
			_tx_data = byte;
		_tx_bitnum = 0;		
		uint16_t tcnt = Timer.gettcnt();
		//*_tx_Port &= _tx_BitMask_inv;		// in case of addtimer fail
		uint8_t rec = Timer.addtimer(tcnt+_bit_delay, _bit_delay, (timer_func)tx_irq, (void*)this);
		if(!_tx_error)
		{
			if(rec<0)
			{
				_tx_error=1;
			}else
			{
				_tx_buffer_empty = 0;
			}
			return 1;
		}else
		{
			if(rec>=0)
			{
				_tx_error=0;
				_tx_buffer_empty = 0;
			}
		}		
	}
	uint8_t next = (_tx_buffer_tail + 1) % _SS_TX_BUFF_SIZE;
	while(next==_tx_buffer_head)
	{
		// Interrupts are disabled, so this function can't work
		if (bit_is_clear(SREG, SREG_I)) 
			return 0;
	}
	_tx_buffer[_tx_buffer_tail] = byte;
	_tx_buffer_tail = next;
	return 1;
}
