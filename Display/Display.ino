///////////////////////////////////////////////////////////////////////////////
// ENGD2001 FIRMWARE
//
// V1.8 Dr J A Gow 22/11/2017
//
// This code supports:
//
//   - Test display
//   - DC Volts (2 and 20V max ranges
//   - added range selector in place ready for the frequency selector
//   - sinusoidal frequency generator
//
///////////////////////////////////////////////////////////////////////////////

// include the library code:

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include "types.h"	// our own types.h overcoming shortcoming in Arduino toolchain
#include "pins.h"   // our own pin definition file

//
// INTERNAL FUNCTION PROTOTYPES
//
// dflt values stop Arduino preprocessor from generating prototype)

void ISRModeChange(void);						// mode change button ISR
void ISRRangeChange(void);						// range change ISR
void display(const MODE mode, float value=0); 	// display function

//
// GLOBAL VARIABLES
//  -- or --
// eat this, computer scientists :-)

LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR,
		              16,2);   		// instantiate one global instance of lcd
MODE mode;						    // this is the selected function mode for the device
volatile BOOLEAN modeChange;	    // used from the mode change ISR to lag input
volatile BOOLEAN rangeChange;		// used from the range change ISR to flag input

//
// Our array of analog ranges. We could have one for each
// rangeable method.

Range DCVoltsRanges[2]={
		Range("(0-2V)",  SCALE_DCV_2VMAX,  RANGE_NO_PIN,    0.0, 2.0),
		Range("(0-20V)", SCALE_DCV_20VMAX, RANGE_RELAY_PIN, 0.0, 20.0)
};

Range ACVoltsRanges[1]={
		Range("(0-20V)", SCALE_ACV_20VMAX, RANGE_RELAY_PIN, 0.0, 20.0)
};

// The individual mode classes for the displays.

Mode 			m1("Multimeter");
DCMode 			m2("DC Volts","V",DCVoltsRanges,2);
ACMode       	m3("AC Volts","V",ACVoltsRanges,1);
PWMSine         m4("Sine generator");
PWMSine			m5("Square generator");
PWMFile         m6("Play file");

// in a mode table (use ptrs so the virtual member function overloading works)

Mode * g_modes[7] { &m1,&m2,&m3,&m4,&m5,&m6};

// PWM ISR vars

PWMMode * PWMMode::currentInstance=NULL;
ACMode *  ACMode::currentInstance=NULL;

// operator++ for the MODE enum type
//
// overload/create the increment operator for the MODE. Enums don't have this by
// default and it allows us to do some bounds-checking
// This function will only work if the enum is sequential.

MODE& operator++(MODE& mode)
{
	mode=static_cast<MODE>(static_cast<int>(mode)+1);
	if(mode==MODE::TOP_MODE) {
		mode=MODE::TESTDISPLAY;  // cycle round at the very highest mode
	}
	if(mode==LAST_MODE) {
		mode=MODE::TESTDISPLAY;	 // cycle round if we artificially restrict it
	}
	return mode;
}

///////////////////////////////////////////////////////////////////////////////
// setup()
//
// Run once at power-up or at reset
//
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Initializing..");
  mode=MODE::TESTDISPLAY;

  // We do this the Arduino way - thus it should work across all variants of H/W

  pinMode(LED_BUILTIN,OUTPUT);	// for debug
  digitalWrite(LED_BUILTIN,FALSE);

  pinMode(MODE_CHANGE_PIN,INPUT);
  pinMode(RANGE_RELAY_PIN, OUTPUT);

  DDRD|=0b00010000;


  for(int i=0;i<6;i++) {
	  g_modes[i]->init();
  }

  lcd.clear();
  attachInterrupt(digitalPinToInterrupt(MODE_CHANGE_PIN),ISRModeChange,LOW);
  attachInterrupt(digitalPinToInterrupt(RANGE_CHANGE_PIN),ISRRangeChange,LOW);
}

///////////////////////////////////////////////////////////////////////////////
// loop()
//
// Called from the Arduino framework: this is repeatedly called.
//
///////////////////////////////////////////////////////////////////////////////

void loop()
{
	static unsigned long mctime,rctime;
	static BOOLEAN mcfired=FALSE,rcfired=FALSE;

	// look for a potential mode change

	if(modeChange) {
		if(!mcfired) {
			mcfired=TRUE;
			g_modes[mode]->leaveMode();
			++mode;
			g_modes[mode]->enterMode();
			mctime=millis();
		} else {
			if((millis()-mctime)>MS_KBD_RPT) {
				mcfired=modeChange=FALSE;
				attachInterrupt(digitalPinToInterrupt(MODE_CHANGE_PIN),ISRModeChange,LOW);
			}
		}
	}

	// look for a potential range change

	if(rangeChange) {
		if(!rcfired) {
			rcfired=TRUE;
			g_modes[mode]->rangeChange();
			rctime=millis();
		} else {
			if((millis()-rctime)>MS_KBD_RPT) {
				rcfired=rangeChange=FALSE;
				attachInterrupt(digitalPinToInterrupt(RANGE_CHANGE_PIN),ISRRangeChange,LOW);
			}
		}
	}

	// process the mode.

	g_modes[mode]->update();
	g_modes[mode]->display();
}

//
// CLASS FUNCTIONS

//
// The Range class

///////////////////////////////////////////////////////////////////////////////
// Range::constructor
//
// All this really does is set the internals, then ensure that the pin,
// if used, is correctly enabled
//
///////////////////////////////////////////////////////////////////////////////

Range::Range(const char * range_text, const float scale,
		     const uint8_t pin, const float maximum, const float minimum)
           : range_text(range_text),scale(scale),maximum(maximum),minimum(minimum),pin(pin)
{
	if(pin>0) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin,FALSE);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Range::enterRange
//
// Called if we switch to this range
//
///////////////////////////////////////////////////////////////////////////////

void Range::enterRange(void)
{
	if(pin>0) {
		digitalWrite(pin,TRUE);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Range::leaveRange
//
// Called if we switch to this range
//
///////////////////////////////////////////////////////////////////////////////

void Range::leaveRange(void)
{
	if(pin>0) {
		digitalWrite(pin,FALSE);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Mode::constructor
//
// Constructor for base class
//
///////////////////////////////////////////////////////////////////////////////

Mode::Mode(const char * title="") throw() // default value stops Arduino preprocessor
	 : firstcall(FALSE), title(title)
{
	this->t_len=0;	// position at start of line (leave space for ranges)
}

///////////////////////////////////////////////////////////////////////////////
// Mode::enterMode
//
// Called when the mode is switched in
//
///////////////////////////////////////////////////////////////////////////////

void Mode::enterMode(void) throw()
{
	this->firstcall=FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// Mode::display
//
// Base class: displays title only
//
///////////////////////////////////////////////////////////////////////////////

void Mode::display(void) throw()
{
	// do this once, only on first call.

	if(!firstcall) {
		lcd.clear();
		lcd.setCursor(this->t_len,0);
		lcd.print(this->title);
		firstcall=TRUE;
	}
}

///////////////////////////////////////////////////////////////////////////////
// FloatMode::enterMode
//
// Iterate our ranges, en
//
///////////////////////////////////////////////////////////////////////////////

void FloatMode::enterMode(void) throw()
{
	Mode::enterMode();
	// make sure the current range is set
	if(curRange) {
		curRange->enterRange();
	}
}

///////////////////////////////////////////////////////////////////////////////
// FloatMode::update
//
// Updates the float value directly from the ADC
//
///////////////////////////////////////////////////////////////////////////////

void FloatMode::update(void) throw()
{
	// we must scale for our current range, if we have one.
	if(curRange)
		value*=curRange->scale;
}

///////////////////////////////////////////////////////////////////////////////
// FloatMode::rangeChange
//
// When the range is changed
//
///////////////////////////////////////////////////////////////////////////////

void FloatMode::rangeChange(void) throw()
{
	if(nRanges>1) {
		++rangeidx;
		if(rangeidx>=nRanges) {
			rangeidx=0;
		}
		curRange->leaveRange();
		curRange=ranges+rangeidx;
		curRange->enterRange();
		this->update();
	}
}

///////////////////////////////////////////////////////////////////////////////
// FloatMode::display
//
// Display the float variable
//
///////////////////////////////////////////////////////////////////////////////

void FloatMode::display(void) throw()
{
	// call the base class display to display the title

	Mode::display();

	// then overlay the float stuff. Scale for display.
	// Display the range first, put the text afterwards

	if(curRange) {
		int s=strlen(this->curRange->range_text);
		lcd.setCursor(15-s,0);
		lcd.print(" ");
		lcd.print(this->curRange->range_text);
	}
	static char chv[16];
	char * cptr=dtostrf(this->value,8,2,chv);
	cptr=(!cptr)?(char *)" -NaN- ":cptr;     // if dtostf fails, display '-NaN-'
	lcd.setCursor((16-strlen(cptr))/2,1);  // centralize on line

	//sprintf(chv,"%lx",(int)this->value);
	//lcd.print(chv);

	lcd.print(cptr);
	lcd.print(this->units);
	lcd.print(" ");
}

///////////////////////////////////////////////////////////////////////////////
// FloatMode::leaveMode
//
// Turn off the scaling I/O pin (if required
//
///////////////////////////////////////////////////////////////////////////////

void FloatMode::leaveMode(void) throw()
{
	if(curRange)
		curRange->leaveRange();
	Mode::leaveMode();
}

///////////////////////////////////////////////////////////////////////////////
// DCMode::update
//
// Updates the float value directly from the ADC
//
///////////////////////////////////////////////////////////////////////////////

void DCMode::update(void) throw()
{
	this->value=analogRead(ANALOG_IN);
	FloatMode::update();
}

///////////////////////////////////////////////////////////////////////////////
// ACMode::enterMode
//
// We need to set up the ADC here to perform sampling. We use the CTC to
// generate a pulse to trigger the ADC. All we have to do then, is wait for the
// ADC interrupt to come in regularly with a value. The peripherals do
// all the hard work!
//
///////////////////////////////////////////////////////////////////////////////

void ACMode::enterMode(void) throw()
{
	FloatMode::enterMode();
	ACMode::currentInstance=this;

	// Set TIMER1 for CTC mode

	DDRB|=0b00000110;	// two PWM pins set o/p for now

	TCCR1A = 0b00010000;
	TCCR1B = 0b00001010;	// 2mHz clock

	OCR1A=0x1f4;			// 4kHz sample rate.
	OCR1B=0x1f4;			// compare at same

	// Turn off the digital input buffers on A1

	DIDR0 |= 0b00000010;

	// Set up VCC reference, right-justified, and A1 input

	ADMUX  = 0b01000001;

	// Set up autotrigger on TIMER1 Compare Match B

	ADCSRB |= 0b00000101;
	ADCSRA |= 0b10101111;	// enable ADC, INTs and clock ADC at 250kHz

	// ints now enabled - analog running.

}

///////////////////////////////////////////////////////////////////////////////
// ACMode::update
//
// Updates the float value directly from the ADC _for now.
//
///////////////////////////////////////////////////////////////////////////////

void ACMode::update(void) throw()
{
	float fromint;

	// this is to ensure that internal values used for display don't
	// get munged by the interrupt, which can occur at any time.

	asm ("cli");	// make the update of display value atomic.
	fromint=(float)(this->ival);
	asm ("sei");

	// this is the 'root' part of the 'root-mean-square' algorithm,
	// remember 'fromint' is in 1.16 notation, normalized to 1,
	// so we need to make a correction.
	// We are not in interrupt context here, so speed less critical

	this->value=sqrt(fromint/32767);

	FloatMode::update();
}

///////////////////////////////////////////////////////////////////////////////
// ACMode::leaveMode
//
// Shut down the CTC and analog.
//
///////////////////////////////////////////////////////////////////////////////

void ACMode::leaveMode(void) throw()
{
	// turn off timer and ADC (doesn't matter if timer stays running)

	// CHECK PRESCALER and add to DCMode::leaveMode.

	ADCSRA = 0b10010110; // turn off ints, clear ADIF but leave ADC enabled.

	// The Arduino libs set the prescalers in the init - this means that
	// if we have changed them, we need to leave them at a setting that
	// will work with analogRead.

	ACMode::currentInstance=NULL;
	FloatMode::leaveMode();
}

///////////////////////////////////////////////////////////////////////////////
// ACMode::ADCInterrupt
//
// INTERRUPT
//
// ADC conversion complete
//
///////////////////////////////////////////////////////////////////////////////

void ACMode::ADCInterrupt(void) throw()
{
	int ival=(ADC<<6)-0x7fff;

	// this will now effectively be scaled to 1.16.

	// we need to calculate either RMS or peak values.

	// FOR RMS:

	PORTD|=0b00010000;

	ival=MULS1616(ival,ival);		// square the input
	this->ival=FILTER2(ival);		// 6th-order Butterworth filter at fc=20Hz

	// We do this to guard against a negative result due to rounding
	// errors at very low input levels.

	this->ival=(this->ival<0)?0:this->ival;

	PORTD&=~0b00010000;
	TIFR1 |= 0b00100111;	// clear all timer int flags
}

///////////////////////////////////////////////////////////////////////////////
// PWMMode::constructor
//
// Initializes the class (but not the pins)
//
///////////////////////////////////////////////////////////////////////////////

PWMMode::PWMMode(const char * title) throw()
		: Mode(title)
{
}

///////////////////////////////////////////////////////////////////////////////
// PWMMode::enterMode
//
// Enter the PWM mode - initialize the peripheral
//
///////////////////////////////////////////////////////////////////////////////

void PWMMode::enterMode(void)
{
	Mode::enterMode();
	// We want fast PWM mode on channel A
	//
	// For TOP=0x3ff,
	// WGM13 - WGM12 - WGM11 - WGM10
	//   0       1       1       1
	//
	// For TOP=ICR1, (we use this)
	// WGM13 - WGM12 - WGM11 - WGM10
	//   1       1       1       0
	//
	// For TOP=OCR1A,
	// WGM13 - WGM12 - WGM11 - WGM10
	//   1       1       1       1
	//
	// Need COM1A1:A0 = 10

	DDRB|=0b00000110;	// two PWM pins set o/p
	TCCR1B=0b00011001;
	TCCR1A=0b10000010;
	ICR1=0x1ff;
	OCR1A=0xff;

	// The above configuration gives us 30kHz at 9bits!
	// Not bad for a cute little processor..

	PWMMode::currentInstance=this;
	TIMSK1 |= 0b00000001;	// turn on int.

}

///////////////////////////////////////////////////////////////////////////////
// PWMMode::leaveMode
//
// Leave the PWM mode - shut down the peripheral
//
///////////////////////////////////////////////////////////////////////////////

void PWMMode::leaveMode(void)
{
	TIMSK1&=~0b00000001;	// switch off the interrupt
	TCCR1A&=~0b11000000;	// turn off the PWM
	DDRB&=~0b00000110;	    // two PWM pins set back to input

	PWMMode::currentInstance=NULL;
	Mode::leaveMode();		// call the parent class
}

///////////////////////////////////////////////////////////////////////////////
// PWMMode::display
//
// Display the frequency (TO BE CHANGED)
//
///////////////////////////////////////////////////////////////////////////////

void PWMMode::display(void) throw()
{
	// call the base class display to display the title

	Mode::display();
}

///////////////////////////////////////////////////////////////////////////////
// PWMSine::constructor
//
// Constructor for the PWM sine generator class
//
///////////////////////////////////////////////////////////////////////////////

PWMSine::PWMSine(const char * title) throw()
		: PWMMode(title), freq(PWMOSC_MIN_FREQ),u1(0),u2(0),u(0)
{
}

///////////////////////////////////////////////////////////////////////////////
// PWMSine::calcInitStateVars
//
// Private function to get valid initial state variables for the biquad for a
// given frequency and amplitude (u1 and u2)
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::calcInitStateVars(float freq) throw()
{
	float w=2*PI*freq/30000;
	k=(int)(cos(w)*32767);		// 32767 for a 1.16 integer notation
	u1=(int)(cos(-w)*30000);	// ampl. slightly less than 1.0 to allow
	u2=(int)(cos(-2*w)*30000);  // for potential overflow.
}
///////////////////////////////////////////////////////////////////////////////
// PWMSine::enterMode
//
// Initialize the sine generator, then start the PWM
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::enterMode(void) throw()
{
	calcInitStateVars(freq);
	PWMMode::enterMode();
}

///////////////////////////////////////////////////////////////////////////////
// PWMSine::leaveMode
//
// just call the base class to shut down the PWM
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::leaveMode(void) throw()
{
	PWMMode::leaveMode();
}

///////////////////////////////////////////////////////////////////////////////
// PWMSine::rangeChange
//
// Called when someone prods the range change. Update the frequency and
// restart (for now)
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::rangeChange(void) throw()
{
	freq+=PWMOSC_FREQ_STEP;
	if(freq>PWMOSC_MAX_FREQ)
		freq=PWMOSC_MIN_FREQ;
	leaveMode();
	calcInitStateVars(freq);
	enterMode();
}

///////////////////////////////////////////////////////////////////////////////
// PWMSine::display
//
// Displays the frequency alongside the title
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::display(void) throw()
{
	PWMMode::display();

	// then overlay the frequency stuff.

	char chv[16];
	char * cptr=dtostrf(this->freq,4,1,chv);
	lcd.setCursor((16-strlen(cptr))/2,1);  // centralize on line
	lcd.print(cptr);
	lcd.print("Hz ");
}


///////////////////////////////////////////////////////////////////////////////
// PWMSine::PWMInterrupt
//
// INTERRUPT
//
// Called from the master ISR to handle the interrupts for this specific class
// It does waste a few cycles
//
///////////////////////////////////////////////////////////////////////////////

void PWMSine::PWMInterrupt(void) throw()
{
	// This is the clever bit! Really! It's a sinusoidal digital
	// oscillator. Looks (and the implementation is) very simple
	// and quick to run (essential in interrupt context)
	//
	// Simple biquad digital oscillator - two conjugate poles sitting on the
	// z-plane unit circle, at e^(jW) and e^(-jW) respectively, where
	// W=2*pi*f/fs
	//
	// The 'int' variables u1, u2 and u are actually fixed point fractionals,
	// using a 1.15 notation. We _can't_ use floats here. Just not enough
	// grunt in the processor.

	u=-u2+(MULS1616(k,u1)<<1);
	u2=u1;
	u1=u;

	// scale output for 9-bit unipolar pwm

	OCR1A=(unsigned int)((u>>7)+(int)0xff);
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::constructor
//
// Initializes the class (but not the pins)
//
///////////////////////////////////////////////////////////////////////////////

PWMFile::PWMFile(const char * title) throw()
		: PWMMode(title),sdok(FALSE)
{
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::init
//
// Initializes the SD card
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::init(void)
{
	this->sdok=SD.begin(4);
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::enterMode
//
// Initialize the file player, then start the PWM
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::enterMode(void) throw()
{
	this->root=SD.open("/");
	this->root.rewindDirectory();
	this->ce=this->root.openNextFile();
	if(this->ce) {
		while(this->ce&&this->ce.isDirectory()) {
			this->ce.close();
			this->ce=this->root.openNextFile();
		}
	}
	PWMMode::enterMode();
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::leaveMode
//
// just call the base class to shut down the PWM
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::leaveMode(void) throw()
{
	PWMMode::leaveMode();
	this->fileBuf.flush();
	if(this->sdok) {
		if(this->ce)
			this->ce.close();
		if(this->root)
			this->root.close();
	}
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::rangeChange
//
// Called when someone prods the range change.
// Change the file and play.
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::rangeChange(void) throw()
{
	if(this->ce) {
		this->fileBuf.flush();
		this->ce.close();
		this->ce=this->root.openNextFile();
		while(this->ce&&this->ce.isDirectory()) {
			this->ce.close();
			this->ce=this->root.openNextFile();
		}
		if(!this->ce) {
			this->root.rewindDirectory();
			this->ce=this->root.openNextFile();
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::display
//
// Display the filename
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::display(void) throw()
{
	// call the base class display to display the title
	char * s;
	PWMMode::display();
	if(!this->sdok) {
		s="No SD card";
	} else {
		if(this->ce) {
			s=this->ce.name();
		} else {
			s="No files";
		}
	}
	lcd.setCursor(0,1);
	lcd.print(s);lcd.print("  ");
}

//////////////////////////////////////////////////////////////////////////////
// PWMFile::update
//
// Called in a loop to read the current file into the buffer
//
//////////////////////////////////////////////////////////////////////////////

void PWMFile::update() throw()
{
	if(this->ce) {
		fileBuf.fill(this->ce);
	}
}

///////////////////////////////////////////////////////////////////////////////
// PWMFile::PWMInterrupt
//
// INTERRUPT
//
// Called from the master ISR to handle the interrupts for this specific class
// It does waste a few cycles
//
///////////////////////////////////////////////////////////////////////////////

void PWMFile::PWMInterrupt(void) throw()
{
	int u=this->fileBuf.drain();
	OCR1A=(unsigned int)((u>>7)+(int)0xff); // scale to 10bit unsigned (offset)
}

//////////////////////////////////////////////////////////////////////////////
// Buffer::isAvail
//
// Returns true if there are ints in the buffer
//
//////////////////////////////////////////////////////////////////////////////

BOOLEAN Buffer::isAvail(void) throw()
{
	int r;
	asm("cli");
	r=this->nFree;
	asm("sei");
	return r;
}

//////////////////////////////////////////////////////////////////////////////
// Buffer::fill
//
// Fill the buffer from the file. If buffer full, return 0, if nothing
// in file, return -1
//
//////////////////////////////////////////////////////////////////////////////

int	Buffer::fill(File& file) throw()
{
	int nf=this->isAvail();
	int nWrMax=min(nf,((this->buffer+BUF_NMAX_BYTE)-this->w));

	if(file.available()&&nWrMax) {
		int nRead=file.read((void *)(this->w),nWrMax);
		if(nRead!=0&&nRead!=-1) {

			// we need to lock out the interrupt in case nFree is modified

			asm("cli");
			this->nFree-=nRead;
			asm("sei");

			// the rest only updates the write ptr, so there is no overlap
			// with the interrupt context.

			this->w+=nRead;
			if(this->w>this->buffer+BUF_NMAX_BYTE) {
				this->w=buffer;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
// Buffer::drain
//
// INTERRUPT
//
// Return an int from the buffer. If the buffer is empty, return silence.
// Always returns
//
//////////////////////////////////////////////////////////////////////////////

int	Buffer::drain(void) throw()
{
	int v=-32767;
	if((BUF_NMAX_BYTE-this->nFree)>1) {
		v=*((int *)(this->r));
		this->nFree+=2;
		this->r+=2;
		if(this->r>(this->buffer+BUF_NMAX_BYTE)) {
			this->r=this->buffer;
		}
	}
	return v;
}

//////////////////////////////////////////////////////////////////////////////
// Buffer::flush
//
// Clear the buffer, remove all data
//
//////////////////////////////////////////////////////////////////////////////

void Buffer::flush(void) throw()
{
	asm("cli");	// make atomic
	this->w=this->r=this->buffer;
	this->nFree=BUF_NMAX_BYTE;
	asm("sei");
}

///////////////////////////////////////////////////////////////////////////////
// ISRModeChange
//
// INTERRUPT
//
// Called via interrupt if someone prods the button. We debounce at
// task time
//
///////////////////////////////////////////////////////////////////////////////

void ISRModeChange(void)
{
	// we signal the state change (in this case just an increment),
	// but then we kill the interrupt so switch bounce doesn't keep
	// firing us.

	// we only fire once..Mr Bond..

	modeChange=TRUE;
	detachInterrupt(digitalPinToInterrupt(MODE_CHANGE_PIN));
}

///////////////////////////////////////////////////////////////////////////////
// ISRRangeChange
//
// INTERRUPT
//
// Called via interrupt if someone prods the range change button. We debounce
// at task time
//
///////////////////////////////////////////////////////////////////////////////

void ISRRangeChange(void)
{
	// we signal the range change (in this case just an increment),
	// but then we kill the interrupt so switch bounce doesn't keep
	// firing us.

	// we only fire once..Mr Bond..

	rangeChange=TRUE;
	detachInterrupt(digitalPinToInterrupt(RANGE_CHANGE_PIN));
}

///////////////////////////////////////////////////////////////////////////////
// ISRPWM
//
// INTERRUPT
//
// This is the master PWM overflow interrupt. We can't use attachInterrupt
// here as there is _way_ too much overhead. The vector ptr is in rom, so
// we eat the overhead of one level of indirection in order to call into the
// class instance. The class is either a singleton, or only one is active
// at any one time.
//
///////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_OVF_vect)
{
	if(PWMMode::currentInstance)
		PWMMode::currentInstance->PWMInterrupt();
}

///////////////////////////////////////////////////////////////////////////////
// ISR(ADC)
//
// INTERRUPT
//
// Handles conversion complete interrupts
//
///////////////////////////////////////////////////////////////////////////////

ISR(ADC_vect)
{
	if(ACMode::currentInstance)
		ACMode::currentInstance->ADCInterrupt();
}
