.EQU	BUZZ_Out	= PB0	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB1	; BUZZER Input from FC 
.EQU	ADC_Inp		= PB2	; is an analog input for voltage sensing

.EQU	TMR_COMP_VAL = 645	; about 3.1 khz (50% duty cycle) at 4mhz clock source

; r0 is allways 0 for cpse commands
;.def	itmp		= r1	; interrupts temp register
;.def	itmp1		= r2	; interrupts temp register
.def	z0			= r24	; zero reg
.def	itmp_sreg	= r22	; storage for SREG in interrupts
.def	buz_on_cntr	= r23	; 0 - buzzer is beeps until pinchange interrupt occurs. 255 - 84ms beep
.def	tmp			= r16 	; general temp register
.def	tmp1		= r17 	; general temp register
.def	pwm_volume	= r18	; range: 1-19. Variable that sets the volume of buzzer (intervalk when BUZZ_Out in fast PWM is HIGH)
.def	pwm_counter	= r19	; just a counter for fast PWM duty cycle
.def	pwm_pin		= r20	; bit indicates the state of 3khz duty cycle (1-1st half, 0-2nd half)
.def	pwm_dutyfst	= r21	; const value 20. Duty cycle len for fast PWM
; r30 has the flag (no sound)

.CSEG
		rjmp RESET 	; Reset Handler
		reti		;rjmp INT0 	; IRQ0 Handler
		rjmp PC_int	; PCINT0 Handler
		reti		; Timer0 Capture Handler
		reti		; Timer0 Overflow Handler
		reti		;rjmp TmrC_int; Timer0 Compare A Handler
		reti		; Timer0 Compare B Handler
		reti		; Analog Comparator Handler
		rjmp WDT_int; Watchdog Interrupt Handler
		reti		; Voltage Level Monitor Handler
		reti		;rjmp ADC_int; ADC Conversion Handler


RESET: 	
		; first determine why we are here (by power-on or by reset pin goes to low)
		clr r30
		in tmp, RSTFLR
		sbrc tmp, EXTRF ; skip next command if reset occurs not by external reset
		ldi r30, 1 ; we should produce no sound until full power restored (ADC reads 5 volts or more)
		clr tmp
		out RSTFLR, tmp	; reset all reset flags 
		
		ldi tmp, 	(1 << BUZZ_Out)	; set pin as output
		out DDRB,	tmp				; all other pins will be inputs		
		ldi tmp, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		out PUEB,	tmp				; 
		clr	tmp
		out PORTB, tmp				; all pins to LOW

		ldi tmp, high (RAMEND) ; Main program start
		out SPH,tmp ; Set Stack Pointer
		ldi tmp, low (RAMEND) ; to top of RAM
		out SPL,tmp

		; 4Mhz (Leave internal 8 mhz osc with prescaler 2)
		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 2 (4mhz)
		out  CLKPSR, tmp
		; configure watchdog
		;rcall WDT_On ; start watchdog timer...
		
		;***** POWER SAVINGS *****
		; disable analog comparator
		ldi	tmp, (1 << ACD)	; analog comp. disable
		out ACSR, tmp			; disable power to analog comp.

		; Configure Pin Change interrupt for BUZZER input
		ldi tmp, 	(1 << BUZZ_Inp)
		out PCMSK, 	tmp	; configure pin for ext interrupt
		ldi tmp, 	0x01
		out PCICR, 	tmp	; pin change interrupt enable

		; Disable digital pin buffer
		ldi tmp, 	(1 << ADC_Inp)
		out DIDR0, 	tmp	
		;***** END OF POWER SAVINGS *****
#ifndef _TN9DEF_INC_
		; Enable and configure ADC
		ldi tmp, (1 << MUX1) | (0 << MUX0)	; PB2 as ADC input
		out ADMUX, tmp
		rcall ADC_start ; run empty ADC read
#endif
		; configure timer 0 to work in CTC mode (4), no prescaler, enable compare interrupt
		ldi tmp, 0
		out TCCR0A, tmp
		ldi tmp, (1 << WGM02) | (1 << CS00)
		out TCCR0B, tmp
		ldi tmp, (0 << OCIE0A) ; currently disable compare interrupt (seems it is faster to take care of it manually)
		out TIMSK0, tmp
		
		; initialize variables
		ldi pwm_dutyfst, 20	; total len of duty cycle of fast PWM
		ldi pwm_volume, 1	; 1-19 value for volume PWM (high freq PWM)
		clr	buz_on_cntr
		clr z0
		
		sei ; Enable interrupts

;******* MAIN LOOP *******	
MAIN_loop:
		rcall WDT_On ; reset watchdog timer for 1 seconds

; ***** MANUAL PWM ROUTINE ******
		; testing timer and PWM
		; initialize PWM generation registers
		; load Compare register to get 3khz
		; Set OCR0A to 645 - about 3khz at 4mhz clock (50% duty cycle)
		dec buz_on_cntr ; load 255 to the buzzer counter (about 84ms)
		cli
		out TCNT0H, z0	; reset timer just in case...
		out TCNT0L, z0
		ldi tmp1,high(TMR_COMP_VAL)
		ldi tmp,low(TMR_COMP_VAL)
		out OCR0AH,tmp1
		out OCR0AL,tmp
		sei

		ldi pwm_pin, 1	; pin state (bit 0 - on/off)
		; pwm_counter is a counter here
PWM_loop:
		; check pwm_pin for main PWM, should we be low, or fast PWM?
		sbrs pwm_pin, 0	; check bit 0
		rjmp PWM_low ; no pin activity on second half of 3khz duty cycle
		; lets toggle fast Buzzer pin while we in first half of 3khz duty cycle
		clr pwm_counter		; counter for fast PWM
		; Fast PWM
		sbi PORTB, BUZZ_Out ; turn buzzer ON
PWM_loop_fast:
		in tmp, TIFR0
		sbrc tmp, OCF0A ; compare match?
		rjmp PWM_loop_slow	; We reached second half of 3khz duty cycle?
		inc	pwm_counter				; count value for pin ON for Buzzer volume regulation
		cpse pwm_counter, pwm_volume	; need to go low. Go out of the loop if values is equal
		rjmp PWM_loop_fast
		cbi PORTB, BUZZ_Out ; turn buzzer OFF
PWM_loop_fast1:		
		in tmp, TIFR0
		sbrc tmp, OCF0A ; compare match?
		rjmp PWM_loop_slow	; We reached second half of 3khz duty cycle?
		inc	pwm_counter			; continue to count while buzzer pin is off in fast PWM
		cpse pwm_counter, pwm_dutyfst	; go out of the loop if values is equal
		rjmp PWM_loop_fast1
		rjmp PWM_loop	; loop untill exit by Timer Compare match
; no pin change in this loop, only wait for Compare match
PWM_low:
		cbi PORTB, BUZZ_Out	; PWM in low state
PWM_lw1:in tmp, TIFR0
		sbrc tmp, OCF0A ; compare match?
		rjmp PWM_loop_slow
		; TODO: need to exit from all this looping when PWM should be off
		cpse buz_on_cntr, z0
		rjmp chck_pcint		; go to routine to check, does PC_int (pin change interrupt) occurs?
		dec buz_on_cntr		; dec counter for buzzer
		breq PWM_loop_exit	; Stop Buzzer beep
		rjmp PWM_lw1 ; otherwise just wait here. 
; 3khz 50% duty cycle transition		
PWM_loop_slow:		
		inc pwm_pin		; toggle bit 0 in register
		ldi tmp, (1 << OCF0A)
		out TIFR0, tmp	; clear compare match flag manually
		rjmp PWM_loop

; currently do nothing here
chck_pcint:		
		rjmp PWM_lw1
; here we finish our handmade PWM routine for buzzer.
PWM_loop_exit:
	; ***** END OF MANUAL PWM ROUTINE ******
		
		
		rcall GO_sleep ; stops here until wake-up event occurs
		; After waking up we need to read ADC for 2 reasons.
		; 1. We want to know the VCC voltage to adjust PWM to the BUZZER.
		; 2. We need to determine VCC loss (FC powered off), to enable BEAKON mode.
		#ifndef _TN9DEF_INC_
		rcall ADC_start ; read ADC value to r17
		#endif
		
		rjmp MAIN_loop 
;******* END OF MAIN LOOP *******	

PC_int:
		reti
		
; test routine for volume change
WDT_int:
		in itmp_sreg, SREG
		inc pwm_volume
		cp pwm_volume, pwm_dutyfst	; compare to max value for volume allowed (1-19)
		brlo WDTiext ; exit if no counter reset needed
		ldi pwm_volume, 1	; reset volume counter
WDTiext:
		out SREG, itmp_sreg
		reti

;TmrC_int:
;		reti		
	
		
WDT_On:
		wdr					; reset the WDT
		; Clear WDRF in RSTFLR
		in tmp, RSTFLR
		andi tmp, ~(1<<WDRF)
		out RSTFLR, tmp		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		;ldi tmp, (0<<WDE) | (1<<WDIE) | (1 << WDP0) | (0 << WDP1) | (0 << WDP2) | (1 << WDP3) | (1<<WDIF) ; 8 sec, interrupt enable
		ldi tmp, (0<<WDE) | (1<<WDIE) | (0 << WDP0) | (1 << WDP1) | (1 << WDP2) | (0 << WDP3) | (1<<WDIF) ; 1 sec, interrupt enable
		out  WDTCSR, tmp
		wdr
		ret	; End WDT_On
		

GO_sleep:
		; Configure Sleep Mode
		ldi tmp, (1<<SE) | (0<<SM2) | (1<<SM1) | (0<<SM0)	; enable power down sleep mode
		out SMCR, tmp
		SLEEP
		; stops here until wake-up event occurs
		ret
#ifndef _TN9DEF_INC_		
; Configures ADC, starts conversion, waits for result...
; returns result in r17		
ADC_start:
		ldi tmp, (0 << PRADC)
		out PRR, tmp
		ldi tmp, (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIF) | (0 <<  ADIE) ; prescaler 2, auto disabled
		out ADCSRA, tmp
		; wait for adc finish
WaitAdc1:
		; check ADSC bit, conversion complete if bit is zero
		sbic ADCSRA, ADSC ; conversion ready?
		rjmp WaitAdc1 ; not yet
		; read MSB of the AD conversion result
		in r17, ADCL
		; switch AD converter off
		ldi tmp, (0<<ADEN) ; switch ADC off
		out ADCSRA, rmp		
		ldi tmp, (1 << PRADC)
		out PRR, tmp
		ret
#endif
		
