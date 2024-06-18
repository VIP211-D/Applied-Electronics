; MULTIPLIERS.S
;
; V1.8 Dr J A Gow 2017
;
; Contains MUL1616, and three functions forming a 6th-order Butterworth filter#
;
; MUL1616 overcomes a shortcoming in the embedded Arduino maths libraries: - it
; is very difficult to do fixed point arithmetic (specifically multiplies)
; in a reasonable number of cycles.
;
; We need a way to quickly do 1.15 x 1.15
;
; 1.15 is a 16 bit integer. The '1' on the lhs of the '.' refers to the
; number of integer bits, the 15 on the rhs of the '.' are fractional bits.
; As the integer is 2's complement signed, the MSB is actually just the
; sign bit.
;
; Examples:
;
; 0.5 = 0x4000			-0.5 = 0xC000
; 0.25 = 0x2000			-0.25 = 0xE000
; 0.125 = 0x1000		-0.125 = 0xF000
;
; and so on.
;
; 1.15 multiplies generate a 32 bit product, but then need shifting in
; order to ensure the decimal point remains in the right place. Then
; the bottom word (underflow) is discarded and the 1.16 result is from
; the top word.
;
; We need this fast multiplier as the AVR only has an 8-bit multiplier
; on chip, and we need to do 1.15x1.15 multiplies at interrupt time in
; order to realize the digital filters/oscillators
;
; Filters
; -------
;
; The filter functions are:
;   MAC (a Multiply and Accumulate),
;   RNDLIMITER (rounds and limits the accumulator to 1.15)
;   FILTER (implements a single second-order Butterworth section)
;	FILTER2 (cascades 3 second-order sections to make a 6th-order Butterworth)
;
; These functions are described in their headers.
;

#define __SFR_OFFSET	0
#include "avr/io.h"

.global MULS1616
.global	FILTER2

.section	.data

state:		.word 0
			.word 0
			.word 0
			.word 0

state2:		.word 0
			.word 0
			.word 0
			.word 0

state3:		.word 0
			.word 0
			.word 0
			.word 0

.section	.text

;
; This is the coefficient table for a second-order section of a Butterworth RMS filter.
; Stored in program memory (flash) - we access it indirectly through the Z reg
;
; The coefficients are stored in lowest order of z^-1 first, with the
; numerator coefficients first.
;
; The coefficients are for an fc of 20Hz at 4kHz sampling

COEFFS:		.byte 0x00		; a0 (a 24-bit coefficient)
			.byte 0x00		;
			.byte 0x12		;

			.byte 0x00		; a1 (a 24-bit coefficient)
			.byte 0x00
			.byte 0x1a

			.byte 0x00 		; a2 (a 24-bit coefficient)
			.byte 0x00		; 1.00
			.byte 0x12

			.byte 0x01		; b1 (a 24 bit coefficient)
			.byte 0xf4		; 1.95556
			.byte 0xa0

			.byte 0xff		; b2
			.byte 0x0b		; -0.95654
			.byte 0x20

			.byte 0x00		; not a coefficient
			.byte 0x7f		; this is the maximum value for the
			.byte 0xff		; limiter

			.byte 0xff		; not a coefficient
			.byte 0x80		; this is the minimum value for the
			.byte 0x00		; limiter

			.byte 0x00		; pure padding to align function on word boundary

;
; MULS1616
;
; This is a specialized 1.15 signed fractional multiply.
; It takes two 1.15 args and multiplies, with a 1.15 output
;
; From C: int MULS1616(int a, int b);
;
; Args: r25:r24  r23:r22 (1.15, 1.15) big-endian
; Retval: r25:r24 (1.15) big-endian

; Used internally r19:r18:r17:r16 = r23:r22 *r25:r24
; the fmul(s) functions can't access the higher regs (>23) directly
; so we save r25:r24 in r21:20
; Basic algorithm straight from AVR datasheet.

MULS1616:	push	r16		; compiler uses these regs
			push	r17
			push	r2
			push	r0
			push	r21
			push	r20
			push 	r19
			push	r18

			clr 	r2
			movw	r20,r24
			fmuls	r23,r21
			movw	r18,r0
			fmul	r22,r20
			adc		r18,r2
			movw	r16,r0
			fmulsu	r23,r20
			sbc		r19,r2
			add		r17,r0
			adc		r18,r1
			adc		r19,r2
			fmulsu	r21,r22
			sbc		r19,r2
			add		r17,r0
			adc		r18,r1
			adc		r19,r2

			movw	r24,r18	; we only need the upper word

			clr		r1		; compiler likes r1=0

			pop		r18
			pop		r19
			pop		r20
			pop		r21
			pop		r0		; restore the regs for the caller
			pop		r2
			pop		r17
			pop		r16
			ret

;
; MAC
;
; Multiply And Accumulate
;
; This is a useful fixed point function for digital filters. Taking an 8.16
; coefficient in r18:r17:r16, and a 1.16 value in r20:r19, will multiply them
; and then accumulate the result in 9.31 format to r25:r24:r23:r22:r21. On entry,
; r25:r24:r23:r22:r21 must contain the accumulator value to which the result will be
; added.
;
; r1 and r0 are not saved.
;
; Function is NOT C-callable
;
;

MAC:		clr r2
			clc

			mul r19,r16
			add r21,r0
			adc	r22,r1			; r15 clear - need to sum for ovf
			adc	r23,r2			; add in any carry,
			adc r24,r2			; rippling through to the higher
			adc r25,r2			; order bytes of the accumulator
			mul r19,r17
			add r22,r0
			adc r23,r1
			adc r24,r2			; add in any carry
			adc r25,r2			; ripple through to high byte of acc.
			mulsu r18,r19
			sbc r25,r2			; carry indicates sign, if -ve subtract it.
			add r23,r0
			adc r24,r1
			adc r25,r2			; add and ripple through the carry.
			mulsu r20,r16
			sbc r24,r2
			sbc r25,r2			; ripple carry through to upper byte
			add r22,r0
			adc r23,r1
			adc r24,r2
			adc r25,r2
			mulsu r20,r17
			sbc r25,r2
			add r23,r0
			adc r24,r1
			adc r25,r2
			muls r18,r20
			add r24,r0
			adc r25,r1			; result in r25:r24:r23:r22:r21

			ret

;
; RNDLIMITER
;
; Firstly this function rounds the 40-bit accumulator to a 24 bit
; accumulator - this is done by rotating the MSB of r22 into the carry,
; then summing this bit into the rest of the registers.
;
; Then, this function takes the 40-bit accumulator and limits according
; to the two limits in the array pointed to by the Z-register on
; entry. These are 24-bit limits. If the limits are such that the upper
; byte is never used, the result is a 1.16 in in r24 and r23.

RNDLIMITER:	; now round the answer (as we have to truncate).
			; this corrupts r22 in the process but we no longer
			; need r22 and r21 after this.

			clr		r2
			lsl		r22			; shove the MSB in the carry flag
			adc		r23,r2		; add it in to the above bytes
			adc		r24,r2		; this works for -ve numbers
			adc		r25,r2
			clr		r22
			clr		r21			; chuck this away.

			; place limiter on output - only worry about
			; the 9.15 portion of the accumulator

			lpm		r18,Z+		; load maximum value for limiter
			lpm		r17,Z+
			lpm		r16,Z+

			cp		r23,r16
			cpc		r24,r17
			cpc		r25,r18

			brlt	neglimit

			mov		r25,r18
			mov		r24,r17
			mov		r23,r16

			jmp		nolimit

neglimit:	lpm		r18,Z+
			lpm		r17,Z+
			lpm		r16,Z+

			cp		r23,r16
			cpc		r24,r17
			cpc		r25,r18

			brge	nolimit

			mov		r25,r18
			mov		r24,r17
			mov		r23,r16

nolimit:	ret


;
; FILTER
;
; Arg (input) is in r25:r24 in 1.16, return value is in r25:r24
; allowing for easy cascade.
; No registers are saved and function is NOT C-callable
;
; Entry: Y points to a block of four words for the state vars in SRAM.
;        Z points to the coefficient and limiter block in program memory
;
; Exit:	 Y does not change
;		 Z points beyond end of coefficient/limiter block
;
; Uses:	r25,r24,r23,r22,r21,r19,r18,r17,r16,r15,r14

FILTER:		movw	r14,r24		; save input for z^-1

			mov		r20,r25
			mov		r19,r24		; input - capture before used for acc.

			clr		r25
			clr		r24
			clr		r23
			clr		r22
			clr		r21			; clear 40-bit accumulator

			lpm		r18,Z+		; load a0
			lpm 	r17,Z+
			lpm 	r16,Z+
			rcall 	MAC			; u=in*a0

			ldd		r20,Y+0
			ldd		r19,Y+1
			lpm		r18,Z+
			lpm		r17,Z+
			lpm		r16,Z+
			rcall	MAC			; u=in*a0+in*z^-1*a1

			ldd		r20,Y+2
			ldd		r19,Y+3
			lpm		r18,Z+
			lpm		r17,Z+
			lpm		r16,Z+
			rcall	MAC			; u=in*a0+in*z^-1*a1+in*z^-2*a2


			ldd		r20,Y+4
			ldd		r19,Y+5
			lpm 	r18,Z+		; load b1
			lpm		r17,Z+
			lpm 	r16,Z+
			rcall 	MAC			; u=in*a0+in*z^-1*a1+in*z^-2*a2 +
			                    ;   out*z^-1*b1

			ldd		r20,Y+6
			ldd		r19,Y+7
			lpm 	r18,Z+		; load b2
			lpm		r17,Z+
			lpm 	r16,Z+
			rcall	MAC			; u=in*a0+in*z^-1*a1+in*z^-2*a2
			                    ;    + out*z^-1*b1 + out*z^-2*b2

			; now round and limit: Z-reg points to 24 bit upper and
			; lower limits in order

			rcall	RNDLIMITER

			; we are now working with a 1.16 in r24:r23
			; we can now use r18,17,16 for anything else

			; here we execute the z^-1 operators

			ldd		r17,Y+0
			std		Y+2,r17
			ldd		r17,Y+1
			std		Y+3,r17
			std		Y+0,r15
			std		Y+1,r14

			ldd		r17,Y+4
			std		Y+6,r17
			ldd		r17,Y+5
			std		Y+7,r17
			std		Y+4,r24
			std		Y+5,r23

			mov		r25,r24		; output of accum to move to output
			mov		r24,r23		; accum: r24:r23 in 1.15 .

			ret

;
; FILTER2
;
; This is the main entry point for the 6th-order Butterworth filter
; formed from three cascaded 2nd-order sections. This has all regs saved
; and _is_ C-callable. Execution time is about 82us at 16MHz, more than
; enough time for a 4kHz sample rate
;
; We do it this way because to implement a sixth-order Butterworth
; directly in one section would require incredibly low numerator coefficients,
; and this would mean either extending the numerator coefficients to compensate
; or having unacceptable coefficient rounding error. We get this to some
; degree with the second-order sections.
; We _could_ make a more efficient job of this by using a different
; multiplier for the numerator coefficient/state variable MACs, but this
; is not worthwhile here.
;
; Input: r25:r24 in 1.15 format
; Output: r25:r24 in 1.15 format
;

FILTER2:	push	r0
			push 	r1
			push 	r2
			push	r12
			push	r13
			push	r14
			push	r15
			push	r16
			push 	r17
			push	r18
			push	r19
			push	r20
			push	r21
			push	r22
			push	r23
			push	ZH
			push	ZL
			push	YH
			push    YL


			; Load the y-pointer with the internal state variable table

			ldi		YH, hi8(state)
			ldi		YL, lo8(state)

			; Load the z-pointer with the coefficient and limit table

			ldi		ZH, hi8(COEFFS)
			ldi		ZL, lo8(COEFFS)

			call	FILTER			; call the first pass of the filter

			; Load the y-pointer with the internal state variable table

			ldi		YH, hi8(state2)
			ldi		YL, lo8(state2)

			; Load the z-pointer with the coefficient and limit table

			ldi		ZH, hi8(COEFFS)
			ldi		ZL, lo8(COEFFS)

			call	FILTER			; second pass, making 4th order

			; Load the y-pointer with the internal state variable table

			ldi		YH, hi8(state3)
			ldi		YL, lo8(state3)

			; Load the z-pointer with the coefficient and limit table

			ldi		ZH, hi8(COEFFS)
			ldi		ZL, lo8(COEFFS)

			call	FILTER			; third pass, making 6th order

			pop 	YL
			pop		YH
			pop		ZL
			pop		ZH
			pop		r23
			pop		r22
			pop		r21
			pop		r20
			pop		r19
			pop		r18
			pop 	r17
			pop		r16
			pop		r15
			pop		r14
			pop		r13
			pop		r12
			pop		r2
			pop		r1
			pop		r0
			ret

