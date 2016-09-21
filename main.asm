.EQU	BUZZ_Out	= PB0	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB1	; BUZZER Input from FC 
.EQU	ADC_Inp		= PB2	; is an analog input for voltage sensing

.EQU	TMR_COMP_VAL = 645 - 30	; about 3.1 khz (50% duty cycle) at 4mhz clock source

.undef ZL
.undef ZH
; r0-r15 is not available in this tiny mcu series.
.def	tmp			= r16 	; general temp register
.def	tmp1		= r17 	; general temp register
.def	pwm_volume	= r18	; range: 1-19. Variable that sets the volume of buzzer (intervalk when BUZZ_Out in fast PWM is HIGH)
.def	pwm_counter	= r19	; just a counter for fast PWM duty cycle
.def	pwm_dutyfst	= r20	; const value 20. Duty cycle len for fast PWM
.def	buz_on_cntr	= r21	; 0 - buzzer is beeps until pinchange interrupt occurs. 255 - 84ms beep
.def	adc_val		= r22	; Here we allways have fresh voltage reading value
.def	itmp_sreg	= r23	; storage for SREG in interrupts
.def	tcompL		= r24
.def	tcompH		= r25

;.def	itmp		= r24	; interrupts temp register
;.def	itmp1		= r25	; interrupts temp register
.def	mute_buzz	= r30	; flag indicates that we need to mute buzzer (after reset manually pressed).
.def	z0			= r31	; zero reg
; r30 has the flag (no sound)

.CSEG
		rjmp RESET 	; Reset Handler
		reti		;rjmp INT0 	; IRQ0 Handler
		rjmp PC_int	; PCINT0 Handler
		reti		; Timer0 Capture Handler
		reti		; Timer0 Overflow Handler
		set			; Timer0 Compare A Handler (save time for rcall). exits by next reti command
		reti		; Timer0 Compare B Handler
		reti		; Analog Comparator Handler
		rjmp WDT_int; Watchdog Interrupt Handler
		reti		; Voltage Level Monitor Handler
		reti		;rjmp ADC_int; ADC Conversion Handler


RESET: 	
		; first determine why we are here (by power-on or by reset pin goes to low)
		clr mute_buzz
		in tmp, RSTFLR
		sbrc tmp, EXTRF ; skip next command if reset occurs not by external reset
		ldi mute_buzz, 1 ; we should produce no sound until full power restored (ADC reads 5 volts or more)

		; initialize variables
		clr z0				; general 0 value register
		ldi pwm_dutyfst, 20	; total len of duty cycle of fast PWM
		ldi pwm_volume, 19	; 1-19 value for volume PWM (high freq PWM)
		clr	buz_on_cntr
		ldi tcompL, low(TMR_COMP_VAL)
		ldi	tcompH, high(TMR_COMP_VAL)

		out RSTFLR, z0	; reset all reset flags 
		
		ldi tmp, 	(1 << BUZZ_Out)	; set pin as output
		out DDRB,	tmp				; all other pins will be inputs		
		ldi tmp, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		out PUEB,	tmp				; 
		out PORTB, z0				; all pins to LOW

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
		;ldi tmp, 	(1 << PCIE0)
		;out PCICR, 	tmp	; pin change interrupt enable
		sbi PCICR, PCIE0	; pin change interrupt enable

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
		out TCCR0A, z0
		ldi tmp, (1 << WGM02) | (1 << CS00)
		out TCCR0B, tmp
		ldi tmp, (1 << OCIE0A) ; enable compare interrupt and set in the interrupt routine T flag
		out TIMSK0, tmp
		; disable timer0 for now
		in tmp, PRR
		cbr tmp, PRTIM0
		out PRR, tmp
				
		sei ; Enable interrupts

;******* MAIN LOOP *******	
MAIN_loop:
		rcall WDT_On ; reset watchdog timer for 1 seconds

; ***** MANUAL PWM ROUTINE ******
		; testing timer and PWM
		; initialize PWM generation registers
		; load Compare register to get 3khz
		; Set OCR0A to 645 - about 3khz at 4mhz clock (50% duty cycle)
		ldi buz_on_cntr, 255 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP
		
		rcall GO_sleep ; stops here until wake-up event occurs
		; After waking up we need to read ADC for 2 reasons.
		; 1. We want to know the VCC voltage to adjust PWM to the BUZZER.
		; 2. We need to determine VCC loss (FC powered off), to enable BEAKON mode.
		#ifndef _TN9DEF_INC_
		rcall ADC_start ; read ADC value to adc_val
		#endif
		
		rjmp MAIN_loop 
;******* END OF MAIN LOOP *******	

PC_int:
		reti
		
; test routine for volume change
WDT_int:
		in itmp_sreg, SREG
		;inc pwm_volume
		;cp pwm_volume, pwm_dutyfst	; compare to max value for volume allowed (1-19)
		;brlo WDTiext ; exit if no counter reset needed
		;ldi pwm_volume, 1	; reset volume counter
		;subi tcompL, 2
		;sbc tcompH, z0
WDTiext:
		out SREG, itmp_sreg
		reti

; sets T flag on timer compare match interrupt to speedup manual PWM routine
; routine in the interrupts vector
;TmrC_int:
;		set	; set T flag. No other flags in SREG is affected
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
; returns result in adc_val		
ADC_start:
		; enable ADC
		in tmp, PRR
		cbr tmp, PRADC
		out PRR, tmp
		ldi tmp, (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIF) | (0 <<  ADIE) | (0 <<  ADPS2) | (1 <<  ADPS1) | (1 <<  ADPS0) ; prescaler 8, auto disabled
		out ADCSRA, tmp
WaitAdc1:
		; check ADSC bit, conversion complete if bit is zero
		sbic ADCSRA, ADSC ; conversion ready?
		rjmp WaitAdc1 ; not yet
		; read AD conversion result
		in adc_val, ADCL
		; start second (clean) reading...
		out ADCSRA, tmp
WaitAdc2:
		; check ADSC bit, conversion complete if bit is zero
		sbic ADCSRA, ADSC ; conversion ready?
		rjmp WaitAdc2 ; not yet
		; read AD conversion result
		in adc_val, ADCL
		; switch AD converter off
		out ADCSRA, z0
		ldi tmp, (1 << PRADC)
		out PRR, tmp
		ret
#endif
		
; Beep the buzzer.
;variable buz_on_cntr determines, will routine beep until PCINT cbange interrupt (0 value), or short beep - max 84ms (255 value)
BEEP:
		; enable timer0
		in tmp, PRR
		sbr tmp, PRTIM0
		out PRR, tmp
		cli
		mov tmp1, tcompH
		mov tmp, tcompL
		sei
		out OCR0AH,tmp1
		out OCR0AL,tmp
		out TCNT0H, z0	; reset timer just in case...
		out TCNT0L, z0
		clt		; clear T flag

		; pwm_counter is a counter here
PWM_loop:
		; lets toggle fast Buzzer pin while we in first half of 3khz duty cycle
		; Fast PWM
		sbi PORTB, BUZZ_Out 			;1 turn buzzer ON
		mov pwm_counter, pwm_volume		; Initialize counter
PWM_loop_fast: ; 4 cpu cycles per loop
		brts PWM_low					;1(2) Jump out if T flag is set (Compare Match)
		dec pwm_counter					;1 count value for pin ON for Buzzer volume regulation
		brne PWM_loop_fast				;2
		
		mov pwm_counter, pwm_dutyfst	;1 initialize couner for remaining cycle
		sub pwm_counter, pwm_volume		;1 adjust counter to correct value
		cbi PORTB, BUZZ_Out 			;1 turn buzzer OFF
PWM_loop_fast1:	; 4 cpu cycles per loop
		brts PWM_low					;1(2) Jump out if T flag is set (Compare Match)
		dec pwm_counter					;1 count value for pin OFF for Buzzer volume regulation
		brne PWM_loop_fast1				;2
		rjmp PWM_loop					;2 loop untill exit by Timer Compare match
; no pin change in this loop, only wait for Compare match
PWM_low:	; now second part of 3khz duty cycle
		cbi PORTB, BUZZ_Out	; PWM in low state
		clt		; reset timer capture match flag
		; here we can call ADC read routine once to update voltage readings...
		#ifndef _TN9DEF_INC_
		rcall ADC_start		; we have plenty of time here, so, lets read ADC...
		#endif
PWM_lw1:
		brts PWM_loop_cycle_end				;1(2) Jump out if T flag is set (Compare Match)
		rjmp PWM_lw1 ; otherwise just wait here. 
; 3khz 50% duty cycle transition		
PWM_loop_cycle_end:	; we come here after every timer compare match interrupt	
		clt				; clear T flag ("Compare Match" flag clear)
		cp buz_on_cntr, z0
		breq chck_pcint		; go to routine to check, does PC_int (pin change interrupt) occurs?
		dec buz_on_cntr
		breq PWM_loop_exit	; Stop Buzzer beep
		rjmp PWM_loop

; currently do nothing here
chck_pcint:		
		; we also need to check voltage readings for voltage drop, if, for example, power will be disconnected while beep...
		rjmp PWM_loop
; here we finish our handmade PWM routine for buzzer.
PWM_loop_exit:
		; disable the timer
		in tmp, PRR
		cbr tmp, PRTIM0
		out PRR, tmp
; ***** END OF MANUAL PWM ROUTINE ******
ret
