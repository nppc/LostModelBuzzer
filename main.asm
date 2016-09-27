/*
 * Author: nppc
 * Hardware design: nppc
 * Contributor to the hardware design: universam
 * 
 * This file is the main routine for the Lost Alarm Buzzer module
 *
 * Lost Alarm Buzzer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lost Alarm Buzzer software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
.EQU	BUZZ_Out	= PB0	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB1	; BUZZER Input from FC 
.EQU	ADC_Inp		= PB2	; is an analog input for voltage sensing

.EQU	TMR_COMP_VAL 	= 645 - 30	; about 3.1 khz (50% duty cycle) at 4mhz clock source
.EQU    ADC_LOW_VAL  	= 190      	; adc value lower is Beacon mode
.EQU    ADC_BEACON_VAL  = 150		; below this ADC value we should switch to Beacon mode (less than 1 volt)
.EQU	PWM_FAST_DUTY	= 20		; Fast PWM (Volume) duty cycle len
.EQU	DEFAULT_VOUME	= 20		; Buzzer volume (1-20)

.undef XL
.undef XH
.undef YL
.undef YH
.undef ZL
.undef ZH
; r0-r15 is not available in this tiny mcu series.
.def	tmp			= r16 	; general temp register
.def	tmp1		= r17 	; general temp register
.def	pwm_volume	= r18	; range: 1-20. Variable that sets the volume of buzzer (interval when BUZZ_Out in fast PWM is HIGH)
;.def	pwm_dutyfst	= r19	; const value 20. Duty cycle len for fast PWM
.def	pwm_counter	= r20	; just a counter for fast PWM duty cycle
.def	buz_on_cntr	= r21	; 0 - buzzer is beeps until pinchange interrupt occurs. 255 - 84ms beep
.def	adc_val		= r22	; Here we allways have fresh voltage reading value
.def	itmp_sreg	= r23	; storage for SREG in interrupts
.def	W1_DATA_L	= r24	; L data register for data, received by 1Wire protocol
.def	W1_DATA_H	= r25	; H data register for data, received by 1Wire protocol
.def	icp_d		= r26	; delay from ICP routine (len of the signal)
;.def	wdt_cntr	= r27.  ; counter for wdt interrupts
;.def	itmp1		= r25	; interrupts temp register
.def	mute_buzz	= r30	; flag indicates that we need to mute buzzer (after reset manually pressed).
.def	z0			= r31	; zero reg
; r30 has the flag (no sound)

.DSEG
.ORG 0x0040	; start of SRAM data memory
RST_OPTION: 	.BYTE 1	; store here count of reset presses after power-on to determine special modes of operation
COMP_VAL_RAM_L:	.BYTE 1	; storage for freq value of buzzer. Can be changed in configuration mode.
COMP_VAL_RAM_H:	.BYTE 1 ; storage for freq value of buzzer. Can be changed in configuration mode.
VOLUME_RAM:		.BYTE 1 ; storage for volume value (1-20)

.CSEG
		rjmp RESET 	; Reset Handler
		reti		;rjmp INT0 	; IRQ0 Handler
		reti            ; just wake up... rjmp PC_int	; PCINT0 Handler
		reti		;rjmp TCAP_int; Timer0 Capture Handler
		set			; Timer0 Overflow Handler try T flag first
		set			; Timer0 Compare A Handler (save time for rcall). exits by next reti command
		reti		; Timer0 Compare B Handler
		reti		; Analog Comparator Handler
		reti		;rjmp WDT_int; Watchdog Interrupt Handler
		reti		; Voltage Level Monitor Handler
		reti		;rjmp ADC_int; ADC Conversion Handler

RST_PRESSED: ; we come here when reset button is pressed
		; logic will be:
		; very first action - is just wait for 100ms for example, to eliminate noise on reset button
		; beep shortly n times
		; Then, increment RST_OPTION variable and wait about 2 seconds in loop.
		; if during that time RESET was pressed again, then variable will just increase.
		; After 2 seconds of waiting check RST_OPTION variable and decide what to do.
		; Currently I think about this options:
		; 1. disable buzzer. After battery is disconnected, if user press reset, then it disables beakon functionality until batery is connected back.
		; 2. configure LostModelBuzzer. with some 1wire simple protocol change parameters. (Actually just test them, then reflash, because of no EEPROM for config).
		;    configurable: Buzzer freq, delay for start beakon after power loss, loudness of the buzzer (fixed or adjustad by voltage).
		rcall WAIT100MS
		; if we are not powered from battery, do only buzzer mute 
		rcall ADC_start ; read ADC value to adc_val
		cpi adc_val, ADC_LOW_VAL  	; if battery is not connected
		brsh L1_RST
		sts RST_OPTION, z0			; clear RESET counter to stay in first option 
		; increment counter
L1_RST:	lds tmp, RST_OPTION
		inc tmp
		; loop if pressed too much times
		cpi tmp, 3			; 3 is non existing mode
		brne SKP_OPT_LOOP
		ldi tmp, 1			; gp back to option 1
SKP_OPT_LOOP:
		sts RST_OPTION, tmp
		; beep n times according to RST_OPTION
L1_BUZ_RST:
		push tmp
		ldi buz_on_cntr, 100 ; load 100 to the buzzer counter (about 30ms)
		rcall BEEP_ON		; ignore mute flag
		rcall WAIT100MS
		pop tmp
		dec tmp
		brne L1_BUZ_RST
		;wait 2 seconds more...
		ldi tmp, 40
L1_RST_WAIT:
		push tmp
		rcall WAIT50MS
		pop tmp
		dec tmp
		brne L1_RST_WAIT
		
		; now decide to what mode to go
		; do we just turn off buzzer?
		lds tmp, RST_OPTION
		cpi tmp, 1
		breq RST_BUZZ_OFF
		; if no buzz off then go to 1 wire transfer mode
		; also go to 1 wire mode, if we are powered from battery
		
		; TODO 1wire protocol
		; configure timer0 for capturing 1w data
		; TCCR0A, TCCR0B and TCCR0C is already configured
W1_L0:	ldi tmp, (1 << TOIE0); enable Overflow interrupt 
		out TIMSK0, tmp
		ldi tmp, (1 <<ICF0); clear ICF flag
		out TIFR0, tmp
		clt		; clear T flag 
		rcall TIMER_ENABLE	; enable timer0 and reset timer counter
		; Now wait for data (first need to catch timer overflow, indicating that all transfers are finished)
W1_L1:	; Oveflow will be indicated by T flag is set
		brtc W1_L1		; loop here until overflow will come
		clt				; reset overflow flag
		clr W1_DATA_L	; prepare register for receiving 16 bit of data
		clr W1_DATA_H	; prepare register for receiving 16 bit of data
		ldi tmp1, 16	; counter for receiving bits
		; now wait for the first ICP, it will be the beginning of the first bit.		
W1_L2:	brts W1_L0		; start over, desync or no data came
		in tmp, TIFR0
		sbrs tmp, ICF0
		rjmp W1_L2		; loop
		; read ICP register
		in icp_d, ICR0L
		in icp_d, ICR0H	; we are interested in High byte only
		ldi tmp, (1 <<ICF0)	; clear ICF flag
		out TIFR0, tmp		; clear ICF flag
		; clear timer0 to get new timing... need to check, maybe it is does automatically
		out TCNT0H, z0
		out TCNT0L, z0
		; timing to check
		;low bit - 4ms = 64, 6ms = 96
		;high bit - 9ms = 144, 11ms = 176
		cpi icp_d, 64
		brlo W1_L0		; desync or corrupted data - start over
		cpi icp_d, 176
		brsh W1_L0		; desync or corrupted data - start over
		cpi icp_d, 96
		brlo W1_LOW		; low bit received
		cpi icp_d, 144
		brlo W1_L0		; desync or corrupted data - start over
		; high bit received
		sec				; set C flag
		rjmp W1_CONT
W1_LOW:	clc				; clear C flag
W1_CONT:rol W1_DATA_L	; shift left one bit with C
		rol W1_DATA_H	; shift left one bit with C
		dec tmp1		; go tro next bit
		brne W1_L2		; loop until all data received
		; all data received
		; if high byte is 0, then volume value came, otherwise it is frequency value
		cp W1_DATA_H, z0
		breq W1_ADJ_VOL
		; adjust frequency variable
		STS COMP_VAL_RAM_L, W1_DATA_L
		STS COMP_VAL_RAM_H, W1_DATA_H
		rjmp W1_END
W1_ADJ_VOL:		; Buzzer volume
		STS VOLUME_RAM, W1_DATA_L
W1_END:	; we come here after 16 bits received and proccessed
		rcall TIMER_DISABLE
		; make sample beep
		ldi buz_on_cntr, 255 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP_ON	; skip mute check
		rjmp W1_L0		; back to listen for 1Wire protocol
RST_BUZZ_OFF:
		ldi mute_buzz, 1
		rjmp PRG_CONT	; back to main program

; start of the program
RESET: 	
		cli
		; determine why we are here (by power-on or by reset pin goes to low)
		in tmp1, RSTFLR	; tmp1 should not be changed til sbrc tmp1, EXTRF
		
		; 4Mhz (Leave 8 mhz osc with prescaler 2)
		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 2 (4mhz)
		out  CLKPSR, tmp

		ldi tmp, high (RAMEND) ; Main program start
		out SPH,tmp ; Set Stack Pointer
		ldi tmp, low (RAMEND) ; to top of RAM
		out SPL,tmp

		; initialize variables
		clr z0				; general 0 value register
		;clr wdt_cntr
		;clr	buz_on_cntr		; default is beep until PCINT interrupt
		clr mute_buzz		; by default buzzer is ON
		; default Buzzer frequency
		ldi tmp, low(TMR_COMP_VAL)
		STS COMP_VAL_RAM_L, tmp
		ldi tmp, high(TMR_COMP_VAL)
		STS COMP_VAL_RAM_H, tmp
		; default Buzzer volume
		ldi tmp, DEFAULT_VOUME		; 1-20
		STS VOLUME_RAM, tmp

		; configure pins
		ldi tmp, 	(1 << BUZZ_Out)	; set pin as output
		out DDRB,	tmp				; all other pins will be inputs		
		ldi tmp, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		out PUEB,	tmp				; 
		out PORTB, tmp				; all pins to LOW except pull-up

		; configure timer 0 to work in CTC mode (4), no prescaler
		out TCCR0A, z0
		ldi tmp, (1 << ICNC0) | (0 << ICES0) | (1 << WGM02) | (1 << CS00) ; also preconfigure ICP mode
		out TCCR0B, tmp
		rcall TIMER_DISABLE ; disable timer0 for now

				;***** POWER SAVINGS *****
		; disable analog comparator
		ldi	tmp, (1 << ACD)	; analog comp. disable
		out ACSR, tmp			; disable power to analog comp.

		; Disable digital pin buffer
		ldi tmp, 	(1 << ADC_Inp)
		out DIDR0, 	tmp	
		;***** END OF POWER SAVINGS *****

		; Enable and configure ADC
		ldi tmp, (1 << MUX1) | (0 << MUX0)	; PB2 as ADC input
		out ADMUX, tmp
		;rcall ADC_start ; run empty ADC read

		; Configure Pin Change interrupt for BUZZER input
		ldi tmp, 	(1 << BUZZ_Inp)
		out PCMSK, 	tmp	; configure pin for ext interrupt
		sbi PCICR, PCIE0	; pin change interrupt enable
						
		sei ; Enable interrupts

		out RSTFLR, z0	; reset all reset flags 

		sbrc tmp1, EXTRF ; skip next command if reset occurs not by external reset
		rjmp RST_PRESSED

CHARGE_CAP:
		; here is special startup mode
		; to let supercap to charge above beakon voltage
		rcall ADC_start ; read ADC value to adc_val
		cpi adc_val, ADC_LOW_VAL
		brsh PRG_CONT ; cap is charged
		clr buz_on_cntr ; if pin on, we are ready
		; check input pin for state
		sbis PINB, BUZZ_Inp
		rcall BEEP  ; beep until pin change come
		rjmp CHARGE_CAP
	
PRG_CONT:
		
;******* MAIN LOOP *******	
MAIN_loop:
		; here we should clear SRAM variable, that counts reset presses...
		sts RST_OPTION, z0

		rcall ADC_start ; read ADC value to adc_val
		cpi adc_val, ADC_BEACON_VAL  ; go beakon when voltage divider is relly very low voltage (less than 1 volt)
		brlo GO_BEACON
		
		; check input pin for state
		clr buz_on_cntr ; if pin on, we are ready
		sbis PINB, BUZZ_Inp
		rcall BEEP  ; beep until pin change come
		; go sleep, it will speed up supercap charging a bit...
		rcall WDT_On_8s
		rcall GO_sleep
		; we will wake up on pin change or wdt interrupt
		rjmp MAIN_loop

GO_BEACON:      ; right after power loss we wait a minute, and then beep
		ldi tmp, 8 ; about 1 minute
BEAC_WT1:	
		; TODO-TEST exit here if battery connected back...
		push tmp
		rcall WDT_On_8s
		rcall GO_sleep
		rcall ADC_start ; read ADC value to adc_val
		pop tmp
		cpi adc_val, ADC_LOW_VAL  ; go out of the Beacon mode if power restored
		brsh BEAC_EXIT
		dec tmp
		brne BEAC_WT1
		
BEAC_L1:ldi buz_on_cntr, 200 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP
		rcall WDT_On_250ms	; make small pause...
		rcall GO_sleep ; stops here until wake-up event occurs
		ldi buz_on_cntr, 200 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP
		rcall WDT_On_8s
		rcall GO_sleep ; stops here until wake-up event occur
		rcall ADC_start ; read ADC value to adc_val
		cpi adc_val, ADC_BEACON_VAL ; stay in beakon when voltage divider is relly very low voltage (about 1 volt)
		brlo BEAC_L1
		; go back to main loop - battery connected
		; turn mute off (in case buzzer was muted)
BEAC_EXIT:
		clr mute_buzz	; buzzer should not be muted after going back to normal mode
		;sts RST_OPTION, z0
		rjmp CHARGE_CAP 
;******* END OF MAIN LOOP *******	

	

WDT_On_250ms:
		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (0 << WDP3) | (1 << WDP2) | (0 << WDP1) | (0 << WDP0) ; 0.25 sec, interrupt enable
		rjmp WDT_On
WDT_On_1s:
		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0) ; 1 sec, interrupt enable
		rjmp WDT_On
WDT_On_8s:
		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0) ; 8 sec, interrupt enable
WDT_On:	wdr					; reset the WDT
		; Clear WDRF in RSTFLR
		in tmp, RSTFLR
		andi tmp, ~(1<<WDRF)
		out RSTFLR, tmp		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		out  WDTCSR, tmp1	; preset register
		wdr
		ret	; End WDT_On
		

GO_sleep:
		; Configure Sleep Mode
		ldi tmp, (1<<SE) | (0<<SM2) | (1<<SM1) | (0<<SM0)	; enable power down sleep mode
		out SMCR, tmp
		SLEEP
		; stops here until wake-up event occurs
		ret

WAIT100MS:  ; routine that creates delay 100ms at 4mhz
		rcall WAIT50MS
WAIT50MS:	; routine that creates delay 50ms at 4mhz
		ldi  tmp, 255
		ldi  tmp1, 139
WT50_1: dec  tmp1
		brne WT50_1
		dec  tmp
		brne WT50_1
		ret

; Configures ADC, starts conversion, waits for result...
; returns result in adc_val		
ADC_start:
		; enable ADC
		in tmp, PRR
		cbr tmp, (1 << PRADC) ; clear bit to enable ADC
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
		in tmp, PRR
		sbr tmp, (1 << PRADC) ; set bit to disable ADC
		out PRR, tmp
		ret
		
; Beep the buzzer.
;variable buz_on_cntr determines, will routine beep until PCINT cbange interrupt (0 value), or short beep - max 84ms (255 value)
BEEP:
		cp mute_buzz, z0
		brne PWM_exit		; no sound if flag mute_buzz is set
		; set volume
BEEP_ON:	; call from here if we want to skip beep mute check... 
		lds pwm_volume, VOLUME_RAM ; 1-20 value for volume PWM (high freq PWM)
		;ldi pwm_volume, 20	; 1-20 value for volume PWM (high freq PWM)
		; enable timer0
		rcall TIMER_ENABLE	; reset timer counter
		ldi tmp, (1 << OCIE0A) ; enable compare interrupt 
		out TIMSK0, tmp
		; load Compare register to get 3khz
		; Set OCR0A to 645 - about 3khz at 4mhz clock (50% duty cycle)
		lds tmp1, COMP_VAL_RAM_H
		lds tmp, COMP_VAL_RAM_L
		cli
		out OCR0AH,tmp1
		out OCR0AL,tmp
		sei
		;out TCNT0H, z0	; reset timer just in case...
		;out TCNT0L, z0
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
		
		ldi pwm_counter, PWM_FAST_DUTY	;1 initialize couner for remaining cycle
		sub pwm_counter, pwm_volume		;1 adjust counter to correct value
		breq PWM_loop					;1(2) if volume at max (pwm_dutyfst=pwm_volume) then do not turn buzzer pin low.
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
		cpi buz_on_cntr, 0
		breq chck_pcint		; go to routine to check, does PC_int (pin change interrupt) occurs?
		dec buz_on_cntr
		breq PWM_loop_exit	; Stop Buzzer beep
		rjmp PWM_loop

chck_pcint:		
		; we also need to check voltage readings for voltage drop, if, for example, power will be disconnected while beep...
		sbic PINB, BUZZ_Inp ; stay in loop if pin is low
		rjmp PWM_loop_exit
		cpi adc_val, ADC_LOW_VAL - 30	; we can directly check adc_val, because it is updated in the PWM generation code
		brlo PWM_loop_exit		; go out if beacon mode activated
		rjmp PWM_loop
; here we finish our handmade PWM routine for buzzer.
PWM_loop_exit:
		; disable the timer
		rcall TIMER_DISABLE
; ***** END OF MANUAL PWM ROUTINE ******
PWM_exit:
ret

TIMER_ENABLE:
		; enable the timer
		in tmp, PRR
		cbr tmp, (1 << PRTIM0) ; clear bit
		out PRR, tmp
		; reset timer
		out TCNT0H, z0
		out TCNT0L, z0
		ret

TIMER_DISABLE:
		; disable the timer
		in tmp, PRR
		sbr tmp, (1 << PRTIM0) ; set bit
		out PRR, tmp
		ret
		
