/*
SoftwareSerialEx.h (formerly SoftSerial.h of Arduino) - 
Multi-instance software serial library for Arduino/Wiring
by shangke1988@hust.edu.cn @ 2016/04/26
Tested 19200Hz with one instance

-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)

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
https://github.com/shangke1988/SoftwareSerialEx
*/

#ifndef SoftwareSerialEx_h
#define SoftwareSerialEx_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _TIMER_MAX_ 16

#define _SS_RX_BUFF_SIZE 64 // RX buffer size
#define _SS_TX_BUFF_SIZE 64 // TX buffer size

#define _SS_STOP_BIT 1 // Stop bit number should be more than 1.

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif



typedef uint8_t (*timer_func)(void*);

class HardwareTimer
/*16bit timer heap*/
{
private:
	volatile uint8_t * const _tccra;
	volatile uint8_t * const _tccrb;
	volatile uint16_t * const _tcnt;
	volatile uint16_t * const _ocr;
	volatile uint8_t * const _timsk;
	uint8_t const _bv_ocie;
	volatile uint8_t * const _tifr;
	uint8_t const _bv_ocf;  

	struct {
		uint16_t ocr;
		uint16_t add;
		timer_func func;
		void *target;
	} list[_TIMER_MAX_];
	uint8_t used[_TIMER_MAX_];
	uint8_t heap[_TIMER_MAX_];
	uint8_t count;
public:
	HardwareTimer(volatile uint8_t *tccra, volatile uint8_t *tccrb, volatile uint16_t *tcnt, volatile uint16_t *ocr, volatile uint8_t *timsk, uint8_t ocie, volatile uint8_t *tifr, uint8_t ocf);
	void init();
	int8_t addtimer(uint16_t ocr, uint16_t add, timer_func func, void *target);
	void deltimer(int8_t id);
	inline uint16_t gettcnt(){return *_tcnt;} __attribute__((__always_inline__));
	inline uint8_t getcount(){return count;} __attribute__((__always_inline__));
	void timer_comp_irq();// called by addtimer() and deltimer(), can't be inline
};

extern HardwareTimer Timer;

class SoftwareSerialEx;
class PCINTController
{
private:
	volatile uint8_t * const _pcmsk;
	volatile uint8_t * const _pin;
	SoftwareSerialEx *func_list[8];
	struct{
		uint8_t mask;
		uint8_t bit;
	}mask_list[8];
	uint8_t count;
public:
	PCINTController(volatile uint8_t *pcicr, uint8_t pcie, volatile uint8_t *pcmsk, volatile uint8_t *pin);
	void inline pcint_irq() __attribute__((__always_inline__));
	void inline enable(uint8_t bit) __attribute__((__always_inline__));
	void inline disenable(uint8_t bit) __attribute__((__always_inline__));
	void addpcint(uint8_t bit, SoftwareSerialEx* ss);
	void delpcint(uint8_t bit);
};

class SoftwareSerialEx : public Stream
{
private:
	// [pin data]
	uint8_t _rx_Pin;
	uint8_t _rx_BitMask;
	volatile uint8_t *_rx_Port;
	uint8_t _tx_BitMask,_tx_BitMask_inv;
	volatile uint8_t *_tx_Port;

	// [pcint controller]
	PCINTController *_pci_ctrl;
	uint8_t _pcint_maskvalue;
	uint8_t _pcint_bit;

	// [delay count]
	// Expressed as 1-cycle delays (must never be 0!)
	uint16_t _rx_delay_centering;
	uint16_t _bit_delay;
	
	// [status]
	bool _rx_buffer_overflow;
//	bool _tx_buffer_overflow;
	bool _listen;
	bool _ispcint;

	// [buffer data]
	// There is "volatile" in HardwareSerial.h of Arduino.
	// I don't understand the reason.
	// I think that "volatile" is unnecessary. 
	// We don't care the very short time of the serval Instructions in read() and recv_bits ().
	// Perhaps these functions without "volatile" may run faster
	// "volatile" can't resolve muti-access, unlike cli().
	// Muti-access is unnecessary too.
	// Because only read() change _rx_buffer_head,
	// only recv_bits () change _rx_buffer_tail.
	// Thank you very much for tell me the reason.
	// e-mail: shangke1988@hust.edu.cn
	char _rx_buffer[_SS_RX_BUFF_SIZE]; 
	uint8_t _rx_buffer_tail;
	uint8_t _rx_buffer_head;

	char _tx_buffer[_SS_TX_BUFF_SIZE]; 
	uint8_t _tx_buffer_tail;
	volatile uint8_t _tx_buffer_head;
	uint8_t _tx_buffer_empty;
	uint8_t _tx_error;
	
	// recv data
	uint8_t _rx_bitnum,_rx_data;
	// transmait data
	uint8_t _tx_bitnum,_tx_data;

	// private methods
	void inline enable_pcint() __attribute__((__always_inline__));
	void inline disenable_pcint() __attribute__((__always_inline__));
	
	uint8_t inline recv_bits() __attribute__((__always_inline__));
	uint8_t inline transmit() __attribute__((__always_inline__));
	
public:
	// public methods
	SoftwareSerialEx(uint8_t rx, uint8_t tx);
	~SoftwareSerialEx();
	
	void begin(long speed);
	void listen();
	bool islisten(){return _listen;};
	void stoplisten();

	uint8_t inline recv_start(uint16_t tcnt) __attribute__((__always_inline__));
	uint8_t static rx_irq(SoftwareSerialEx* p);
	uint8_t static tx_irq(SoftwareSerialEx* p);

	bool rx_overflow() { bool ret = _rx_buffer_overflow; if (ret) _rx_buffer_overflow = false; return ret; }
	//bool tx_overflow() { bool ret = _tx_buffer_overflow; if (ret) _tx_buffer_overflow = false; return ret; }
  
	virtual int peek();
	virtual size_t write(uint8_t byte);
	virtual int read();
	virtual int available();
	int availableForWrite(void);

	// I don't know what the function of flush() should be. 
	// flush() is virtual function, but is unsed in the parent class.
	// May be that flush() is reserved for future. 
	// SoftwareSerial.cpp use it as cleaning the rx buffer.
	// In HardwareSerial.cpp, the description is 
	// "This special case is needed since there is no way to force
	// the TXC (transmit complete) bit to 1 during initialization."
	// But I also can't find where flush() is used.
	virtual void flush(){};

	operator bool() { return true; }

	using Print::write;
};
/*
// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round
*/
#endif
