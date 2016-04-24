/*
SoftwareSerialEx.h (formerly SoftSerial.h of Arduino) - 
Multi-instance software serial library for Arduino/Wiring
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

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#define _SS_MAX_TX_BUFF 64 // TX buffer size

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif



typedef uint8_t (*tcfunc)(void*);

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
	tcfunc func;
	void *target;
  } list[_TIMER_MAX_];
  uint8_t used[_TIMER_MAX_];
  uint8_t heap[_TIMER_MAX_];
  uint8_t count;
public:
  HardwareTimer(volatile uint8_t *tccra, volatile uint8_t *tccrb, volatile uint16_t *tcnt, volatile uint16_t *ocr, volatile uint8_t *timsk, uint8_t ocie, volatile uint8_t *tifr, uint8_t ocf);
  int8_t addtimer(uint16_t ocr, uint16_t add, tcfunc func, void *target);
  void deltimer(int8_t id);
  inline uint16_t gettcnt(){return *_tcnt;};
  void timer_comp_irq();
};

extern HardwareTimer Timer;

/*
class SoftwareSerialEx : public Stream
{
private:
  // per object data
  uint8_t _receivePin;
  
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  // buffer data
  // I think that the interrupt only change the tail
  char _receive_buffer[_SS_MAX_RX_BUFF]; 
  volatile uint8_t _receive_buffer_tail;
  uint8_t _receive_buffer_head;
  
  char _transmit_buffer[_SS_MAX_TX_BUFF]; 
  volatile uint8_t _transmit_buffer_tail;
  uint8_t _transmit_buffer_head;
  
  
  
  
  static SoftwareSerialEx *active_object;

  // private methods
  void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state) __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  SoftwareSerialEx(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
  ~SoftwareSerialEx();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }
  
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};

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
