///////////////////////////////////////////////////////////////////////////////
// TYPES.H
//
// Types used by the ENGD2001 multimeter project
//
// V1.8
//
// Dr J A Gow 2017
//
///////////////////////////////////////////////////////////////////////////////

#ifndef TYPES_H_
#define TYPES_H_

// We don't _strictly_ need to put these in a separate file, however it
// overcomes brain-damaged behaviour by the Arduino toolchain - this insists
// on automatically generating function prototypes for you, then stuffing the
// prototype at the BEGINNING of the source file, before you have defined
// the types!!!!
//
// Note that this file contains definitions only. No binary is generated
// by anything in this file alone.

// Boolean type - C/C++ doesn't have one by default

typedef unsigned char BOOLEAN;

#define TRUE	1
#define FALSE	0

// Mode..
//
// Using an enum isn't the only way to do this - but is slightly more 'readable'
// and if we override the increment operator we can make this work well provding it is
// encoded sequentially

typedef enum _MODE {
	TESTDISPLAY =0,
	DCV,
	ACV,
	FUNCTION_SINE,
	FUNCTION_SQUARE,
	SOUND,
	TOP_MODE
} MODE;

#define LAST_MODE	MODE::FUNCTION_SQUARE		// This limits the highest mode that can be instantiated.

//
// From assembler: our multiplier prototype

extern "C" {
	int MULS1616(int a, int b);
	int FILTER2(int a);
}

/////////////
// CLASSES //
/////////////

// Before any C++ purist starts squawking 'there's no destructors' (virtual or otherwise)
// none of these classes will be allocated dynamically, or locally within the stack
// frame of any functions. Ergo, the destructors will never be called. This is, after
// all, an embedded system where the power switch is the 'garbage collector'.
//
// All will be instantiated statically.

//
// Range.
//
// Ranges exist within modes: each mode can have zero or more ranges (where zero, there
// is only one range and no ranges can be selected). That way we can use ranges more
// than once in different modes.

class Range {

	public:
		const char * 	range_text;
		const float		scale;
		const float		maximum;
		const float		minimum;
		const uint8_t	pin;			// set this to -1 if no pin change rqd. Active high.

		Range(const char * range_text, const float scale,
			  const uint8_t pin=0, const float max=0.0, const float min=0.0);
		void enterRange(void);
		void leaveRange(void);
};

//
// Buffer
//
// A specific buffer class for buffering data from the SD card.
// Files are written to the buffer in task time, sent to the ADC in interrupt time

class Buffer {

	#define BUF_NMAX_BYTE	10
	#define BUF_NMAX_INT	(BUF_NMAX_BYTE/2)

	private:

		uint8_t 	buffer[10];
		uint8_t *	w;
		uint8_t *	r;
		volatile int nFree;

	public:

		inline Buffer() : w(buffer),r(buffer),nFree(BUF_NMAX_BYTE) {};
		BOOLEAN isAvail();
		int		fill(File& file);
		int		drain();
		void	flush();
};

// Mode
//
// This little class is a simple way of abstracting a generic display and
// calculation arrangement for all our modes.

class Mode {

	protected:
		BOOLEAN 		firstcall; 	// first time in?

	private:
		const char * 	title;		// the title
		int 			t_len;		// length of title

	public:
		Mode(const char * title) throw();
		inline virtual void init(void) throw() {};		// called on init
		virtual void enterMode(void) throw();			// called on switching to this mode
		virtual inline void leaveMode(void) throw() {};	// called on leaving mode
		virtual inline void rangeChange() throw() {};	// called on range change
		virtual inline void update() throw() {};		// update the mode
		virtual void display(void) throw();
		inline BOOLEAN isFirstCall(void) { return firstcall; }
};

//
// FloatMode
//
// Modes that simply need to display a float from the ADC. It can support

class FloatMode : public Mode {

	private:
		const char * units;
		Range *      ranges;
		int          nRanges;
		Range *      curRange;
		int          rangeidx;

	protected:
		float value;  // the worker child classes need this

	public:
		inline FloatMode(const char * title,
				         const char * units,
						 Range * ranges=NULL,
						 const int nRanges=0) throw()
		                 : Mode(title),units(units),ranges(ranges),
						   nRanges(nRanges),curRange(ranges),rangeidx(0),value(0.0) {};

		virtual void enterMode(void) throw();
		virtual void update() throw();		// scale
		virtual void rangeChange(void) throw();
		virtual void leaveMode(void) throw();
		virtual void display(void) throw();
};

//
// DCMode
//
// Uses FloatMode, but gets its input from an analog input (expecting dc)
// and can offer two ranges. There is no (digital) filtering on the analog input

class DCMode : public FloatMode {

	public:
		inline DCMode(const char * title,
		              const char * units,
				      Range * ranges,
				      const int nRanges) throw()
                    : FloatMode(title,units,ranges,nRanges) {};
		virtual void update() throw();
};

//
// ACMode
//
// Inherits from FloatMode, but uses timers and ADC to get an ac input,
// calculate the rms/peak values, and export these

class ACMode : public FloatMode {

	private:
		volatile int ival;
	public:
		static ACMode * currentInstance;
		inline ACMode(const char * title,
	              const char * units,
			      Range * ranges,
			      const int nRanges) throw()
              : FloatMode(title,units,ranges,nRanges) {};
		virtual void enterMode(void) throw();
		virtual void update() throw();
		virtual void leaveMode(void) throw();
		virtual void ADCInterrupt(void) throw();
};

//
// PwmMode
//
// Class that allows for PWM generation. Note that as the ISRs can only
// access static member functions in a class, this class MUST exist
// only as either (a) a singleton or (b) if derived classes are instantiated,
// each class must be active atomically.

class PWMMode : public Mode {

	private:


	public:

		static PWMMode *	currentInstance;

		PWMMode(const char * title) throw();
		virtual void enterMode(void) throw();
		virtual void leaveMode(void) throw();
		virtual void display(void) throw();
		inline virtual void PWMInterrupt(void) throw() {};
};

//
// PWMSine
//
// Class that builds on PWMMode to create a sinusoidal digital oscillator

class PWMSine : public PWMMode {

	private:

		float freq;		// the 'real' frequency, in Hz

		int	k,u1,u2;
		int u;

		void calcInitStateVars(float freq) throw();

	public:
		PWMSine(const char * title) throw();
		virtual void enterMode(void) throw();
		virtual void leaveMode(void) throw();
		virtual void rangeChange(void) throw();
		virtual void display(void) throw();
		virtual void PWMInterrupt(void) throw();
};

//
// PWMFile
//
// A mode class that plays a file (16 bit, 30kHz sample) from
// an SD card.


class PWMFile : public PWMMode {

	private:
		BOOLEAN sdok;
		File	root;
		File	ce;
		Buffer	fileBuf;

	public:
		PWMFile(const char * title) throw();
		virtual void init(void) throw();
		virtual void enterMode(void) throw();
		virtual void leaveMode(void) throw();
		virtual void rangeChange(void) throw();
		virtual void display(void) throw();
		virtual void update() throw();
		virtual void PWMInterrupt(void) throw();
};


#endif /* TYPES_H_ */
