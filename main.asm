.EQU	BUZZ_Out	= PB0	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB1	; BUZZER Input from FC 
.EQU	ADC_Inp		= PB2	; is an analog input for voltage sensing

.EQU	TMR_COMP_VAL = 645	; about 3.1 khz (50% duty cycle) at 4mhz clock source

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
		in r16, RSTFLR
		sbrc r16, EXTRF ; skip next command if reset occurs not by external reset
		ldi r30, 1 ; we should produce no sound until full power restored (ADC reads 5 volts or more)
		clr r16
		out RSTFLR, r16	; reset all reset flags 
		
		ldi r16, 	(1 << BUZZ_Out)	; set pin as output
		out DDRB,	r16				; all other pins will be inputs		
		ldi r16, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		out PUEB,	r16				; 
		clr	r16
		out PORTB, r16				; all pins to LOW

		ldi r16, high (RAMEND) ; Main program start
		out SPH,r16 ; Set Stack Pointer
		ldi r16, low (RAMEND) ; to top of RAM
		out SPL,r16

		; 4Mhz (Leave internal 8 mhz osc with prescaler 2)
		; Write signature for change enable of protected I/O register
		ldi r16, 0xD8
		out CCP, r16
		ldi r16, (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 2 (4mhz)
		out  CLKPSR, r16
		; configure watchdog
		;rcall WDT_On ; start watchdog timer...
		
		;***** POWER SAVINGS *****
		; disable analog comparator
		ldi	r16, (1 << ACD)	; analog comp. disable
		out ACSR, r16			; disable power to analog comp.

		; Configure Pin Change interrupt for BUZZER input
		ldi r16, 	(1 << BUZZ_Inp)
		out PCMSK, 	r16	; configure pin for ext interrupt
		ldi r16, 	0x01
		out PCICR, 	r16	; pin change interrupt enable

		; Disable digital pin buffer
		ldi r16, 	(1 << ADC_Inp)
		out DIDR0, 	r16	
		;***** END OF POWER SAVINGS *****
		#ifndef _TN9DEF_INC_
		; Enable and configure ADC
		ldi r16, (1 << MUX1) | (0 << MUX0)	; PB2 as ADC input
		out ADMUX, r16
		rcall ADC_start ; run empty ADC read
		#endif
		; configure timer 0 to work in CTC mode (4), no prescaler, enable compare interrupt
		ldi r16, 0
		out TCCR0A, r16
		ldi r16, (1 << WGM02) | (1 << CS00)
		out TCCR0B, r16
		ldi r16, (0 << OCIE0A) ; currently disable compare interrupt (seems it is faster to take care of it manually)
		out TIMSK0, r16
		
		sei ; Enable interrupts

;******* MAIN LOOP *******	
MAIN_loop:
		; testing timer and PWM
		; initialize PWM generation registers
		; load Compare register to get 3khz
		; Set OCR0A to 645 - about 3khz at 4mhz clock (50% duty cycle)
		clr r16
		cli
		out TCNT0H, r16	; reset timer just in case...
		out TCNT0L, r16
		ldi r17,high(TMR_COMP_VAL)
		ldi r16,low(TMR_COMP_VAL)
		out OCR0AH,r17
		out OCR0AL,r16
		sei
		
		ldi r20, 1	; pin state (bit 0 - on/off)
		; r19 is a counter here
		ldi r17, 20	; total len of duty cycle of fast PWM
		ldi r18, 8	; value for volume PWM (high freq PWM)
PWM_loop:
		; check r20 for main PWM, should we be low, or fast PWM?
		sbrs r20, 0	; check bit 0
		rjmp PWM_low ; no pin activity on second half of 3khz duty cycle
		; lets toggle fast Buzzer pin while we in first half of 3khz duty cycle
		clr r19		; counter for fast PWM
		; Fast PWM
		sbi PORTB, BUZZ_Out ; turn buzzer ON
PWM_loop_fast:
		in r16, TIFR0
		sbrc r16, OCF0A ; compare match?
		rjmp PWM_loop_slow	; We reached second half of 3khz duty cycle?
		inc	r19				; count value for pin ON for Buzzer volume regulation
		cpse r19, r18	; need to go low. Go out of the loop if values is equal
		rjmp PWM_loop_fast
		cbi PORTB, BUZZ_Out ; turn buzzer OFF
PWM_loop_fast1:		
		inc	r19			; continue to count while buzzer pin is off in fast PWM
		cpse r19, r17	; go out of the loop if values is equal
		rjmp PWM_loop_fast1
		rjmp PWM_loop	; loop untill exit by Timer Compare match
; no pin change in this loop, only wait for Compare match
PWM_low:
		cbi PORTB, BUZZ_Out	; PWM in low state
PWM_lw1:in r16, TIFR0
		sbrc r16, OCF0A ; compare match?
		rjmp PWM_loop_slow
		; TODO: need to exit from all this looping when PWM should be off
		rjmp PWM_lw1 ; ptherwise just wait here. 
; 3khz 50% duty cycle transition		
PWM_loop_slow:		
		inc r20		; toggle bit 0 in register
		ldi r16, (1 << OCF0A)
		out TIFR0, r16	; clear compare match flag manually
		rjmp PWM_loop
		
		
		rcall WDT_On ; reset watchdog timer for 8 seconds
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
		
WDT_int:
		reti

;TmrC_int:
;		reti		
	
		
WDT_On:
		wdr					; reset the WDT
		; Clear WDRF in RSTFLR
		in r16, RSTFLR
		andi r16, ~(1<<WDRF)
		out RSTFLR, r16		; Write signature for change enable of protected I/O register
		ldi r16, 0xD8
		out CCP, r16
		ldi r16, (0<<WDE) | (1<<WDIE) | (1 << WDP0) | (0 << WDP1) | (0 << WDP2) | (1 << WDP3) | (1<<WDIF) ; 8 sec, interrupt enable
		out  WDTCSR, r16
		wdr
		ret	; End WDT_On
		

GO_sleep:
		; Configure Sleep Mode
		ldi r16, (1<<SE) | (0<<SM2) | (1<<SM1) | (0<<SM0)	; enable power down sleep mode
		out SMCR, r16
		SLEEP
		; stops here until wake-up event occurs
		ret
#ifndef _TN9DEF_INC_		
; Configures ADC, starts conversion, waits for result...
; returns result in r17		
ADC_start:
		ldi r16, (0 << PRADC)
		out PRR, r16
		ldi r16, (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADIF) | (0 <<  ADIE) ; prescaler 2, auto disabled
		out ADCSRA, r16
		; wait for adc finish
WaitAdc1:
		; check ADSC bit, conversion complete if bit is zero
		sbic ADCSRA, ADSC ; conversion ready?
		rjmp WaitAdc1 ; not yet
		; read MSB of the AD conversion result
		in r17, ADCL
		; switch AD converter off
		ldi r16, (0<<ADEN) ; switch ADC off
		out ADCSRA, rmp		
		ldi r16, (1 << PRADC)
		out PRR, r16
		ret
#endif
		
