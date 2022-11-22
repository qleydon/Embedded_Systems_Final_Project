;-------------------------------------------------------------------------------
; MSP430 Assembler Code Template for use with TI Code Composer Studio
; Reads sensor status for wheel left, down, right, up, and center button
; Saves baseline and latest measurements in two arrays
; Updates the status of these sensors in sensor_status
; 	sensor_status can be used to turn on LEDs or trigger other actions
; For wheel left turn on TOP LEFT LED
; For wheel down turn on BOTTOM LEFT LED
; For wheel right turn on BOTTOM RIGHT LED
; For wheel up turn on TOP RIGHT LED
; For wheel center turn on centegetSamplesr LED
; For turning on each of these LEDs you can define constant values that when
; 	loaded in P1OUT will turn on the right LED.  Constant array is shown below.
;	Un-comment to use.
;-------------------------------------------------------------------------------
	.cdecls C,LIST,"msp430.h"       ; Include device header file
SWdelay		.equ	0x0002	; delay value used by the SW timer

			.global waitForCenter
			.global waitForUpDown
			.global waitForLeftRight
			.global getSamples
			.global convertSamples
			.global displaySamples
;-------------------------------------------------------------------------------
; Constant array with the values to turn on LEDs
;-------------------------------------------------------------------------------
;			.sect ".const" ;
;LEDdisplay: 	.byte 0x--		;
;				.byte 0x-- 		;
;				.byte 0x-- 		;
;				.byte 0x-- 		;
;				.byte 0x-- 		;
;-------------------------------------------------------------------------------
; Allocate 10 bytes for the baseline values
;-------------------------------------------------------------------------------
			.data
			.bss Array,40 ;20 words
			.bss LED_Array,40 ;20 words
			.bss	upDown, 1
			.bss	leftRight, 1
			.bss	sensVal, 2
			.bss ADC_SW_FLAG,1 ;
			.bss TAIE_SW_FLAG,1
			.bss TAIE_SW_FLAG_2,1
			.bss	meas_base, 10			;
;-------------------------------------------------------------------------------
; Allocate another 2 bytes for the latest values
;-------------------------------------------------------------------------------
			.bss	meas_latest, 10			;
;-------------------------------------------------------------------------------
; Allocate one byte for sensor status - to be used by the display routine to
; determine which LED to turn on
;-------------------------------------------------------------------------------
			.bss	sensor_status, 1		;
;-------------------------------------------------------------------------------
; Here begins the code segment
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory
            .global RESET
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section
            .retainrefs                     ; Additionally retain any sections
                                            ; that have references to current
                                            ; section
;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer
;-----------------------------------------------------------------------------------------------------------------------------
waitForCenter:
			mov.b	#0,	upDown
			mov.b	#0, leftRight
			mov		#0, sensVal
			bis.b	#0xff, &P1DIR	; set up P1 as outputs
			bic.b	#0xff, &P1OUT	; P1 outputs 0
;-------------------------------------------------------------------------------
			clr	r11
			clr R12
			clr R13
			clr R14
			clr R15

			clr ADC_SW_FLAG
			clr TAIE_SW_FLAG

;-------------------------------------------------------------------------------
Reset_array:
					mov	#0x0000, Array(R13)
					add #2, R13;
					cmp #40, R13
					jne Reset_array
;-------------------------------------------------------------------------------
; The real mainloop starts here
;-------------------------------------------------------------------------------
			call #meas_base_val		; do this once
Mainloop_CN
			;bic.b	#0xff,&P1OUT	; turn out all LEDs
			clr &P1OUT
			mov.b #1, &P1OUT ; turn on led 1
			call #meas_latest_val		;
			mov.b #1, &P1OUT ; turn on led 1
			call #det_sensor		;
			mov.b #1, &P1OUT ; turn on led 1
			call #display
			mov.b #1, &P1OUT ; turn on led 1	;

cmp_CN		cmp.b	#0x0f, sensVal
			jeq	sendback
			jmp	Mainloop_CN

waitForUpDown: ;currently reads all sensors and determines upDown. also turns on top and bottom leds.
			mov.b	#0,	upDown
			mov.b	#0, sensVal
			mov.b	#0, leftRight
			bis.b	#0xff, &P1DIR	; set up P1 as outputs
			bic.b	#0xff, &P1OUT	; P1 outputs 0
;-------------------------------------------------------------------------------
; The real mainloop starts here
;-------------------------------------------------------------------------------
			call #meas_base_val		; do this once
Mainloop_UD
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0x80, &P1OUT	; turn on LED4
			bis.b	#0x10, &P1OUT	; turn on LED1
			call #meas_latest_val		;
			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x80,&P1OUT
			bic.b	#0x10,&P1OUT
			;call	#SWtimer
			;call	#SWtimer
			call	#SWtimer_GS
			call #det_sensor		;
			call #display			;

			cmp.b #1, sensVal
			jeq mov_sensVal_UD
			cmp.b #2, sensVal
			jeq mov_sensVal_UD

cmp_UD		cmp.b	#0, upDown
			jne		sendback
			jmp		Mainloop_UD



waitForLeftRight: ;currently reads all sensors and determines upDown. also turns on top and bottom leds.
			mov.b	#0, leftRight
			mov.b	#0, sensVal
			bis.b	#0xff, &P1DIR	; set up P1 as outputs
			bic.b	#0xff, &P1OUT	; P1 outputs 0
;-------------------------------------------------------------------------------
; The real mainloop starts here
;-------------------------------------------------------------------------------
			call #meas_base_val		; do this once
Mainloop_LR	bic.b	#0xff,&P1OUT	; turn out all LEDs
			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0x40, &P1OUT	; turn on LED3
			bis.b	#0x20, &P1OUT	; turn on LED2
			call #meas_latest_val		;
			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x40,&P1OUT
			bic.b	#0x20,&P1OUT
		;	call	#SWtimer
		;	call	#SWtimer
			call	#SWtimer_GS
			call #det_sensor		;
			call #display			;

			cmp.b #3, sensVal
			jeq mov_sensVal_LR
			cmp.b #4, sensVal
			jeq mov_sensVal_LR

cmp_LR		cmp.b #0, leftRight
			jne sendback
			jmp	Mainloop_LR

;-------------------------------------------------------------------------------
mov_sensVal_UD
			mov.b sensVal, upDown
			jmp cmp_UD
mov_sensVal_LR
			mov.b sensVal, leftRight
			jmp	cmp_LR

SWtimer_GS:
			mov	#0xA, R14		; Load delay value in r11
Reloadr7_GS	mov	#0XA, R13		; Load delay value in R12
ISr70_GS	dec	R13					; Keep this PW for some time*********************************************8
			jnz	ISr70_GS				; The total SW delay count is
			dec	R14					;  = SWdelay * SWdelay
			jnz	Reloadr7_GS			;
			ret						; Return from this subroutine
;------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
; End mainloop ==> all subroutines from here on
;-------------------------------------------------------------------------------
; Routine: Measure base line values
;-------------------------------------------------------------------------------
meas_base_val:	mov.b	#0x02, r11	; initialize r11 to point to P2.x
				mov.b	#0x00, r12	; initialize r12 to the base of meas_base
meas_base_again	call #meas_setup	;
;-------------------------------------------------------------------------------
; Clear TAR and start TA0 in continuous mode; use BIS and not MOV
; so that you don't cancel previous settings
;-------------------------------------------------------------------------------
			bis #MC_2 + TACLR, &TA0CTL 	;
;-------------------------------------------------------------------------------
; Call the SW delay routine, which here it is used to provide the accumulation
; period; could use instead ACLK fed from VLO
;-------------------------------------------------------------------------------
			call #SWtimer			;
;-------------------------------------------------------------------------------
; Now, after the accumulation period has passed, generate a SW based
; capture trigger by toggeling CCIS0
;-------------------------------------------------------------------------------
			xor	#CCIS0, &TA0CCTL1	;
;-------------------------------------------------------------------------------
; Save the baseline captured value in meas_base array
;-------------------------------------------------------------------------------
			mov	TA0CCR1, meas_base(r12)	; note the use of the SYMBOLIC AM
			bic #MC1+MC0, &TA0CTL 	; Stop TA
			sub #2, meas_base(r12)	; Adjust this baseline - YOU MIGHT NEED TO
                                    ;   ADJUST THE #2 VALUE FOR YOUR OWN BOARD!
			bic.b 	r11,&P2SEL2		; Stop the oscillation on the latest. pin
			rla.b	r11				; Prepare next x
			add.b	#0x02, r12		; Prepare the next index into the array
			cmp.b	#0x40, r11		; Check if done with all five sensors
			jne		meas_base_again	;
			ret						;
;-------------------------------------------------------------------------------
; Routine: Measure latest values
;-------------------------------------------------------------------------------
meas_latest_val:
			mov.b	#0x02, r11	; initialize r11 to point to P2.1
			mov.b	#0x00, R12	; initialize R12 to the base of meas_base
meas_latest_again
			call #meas_setup	;
			bis #MC_2 + TACLR, &TA0CTL 	; Continuous, Clear TAR
			call #SWtimer			;
			xor #CCIS0, &TA0CCTL1	; Trigger SW capture
			mov TA0CCR1, meas_latest(R12)	; Save captured value in array
			bic #MC1+MC0, &TA0CTL 	; Stop timer
			bic.b 	r11,&P2SEL2		; Stop the oscillation on the latest. pin
			rla.b	r11				; Prepare next x
			add.b	#0x02, R12		; Prepare the next index into the array
			cmp.b	#0x40, r11		; Check if done with all five sensors
			jne		meas_latest_again	;
			ret						;
;-------------------------------------------------------------------------------
; Routine: Determine which sensor was pressed. Note there is no need for debouncing
;-------------------------------------------------------------------------------
det_sensor:	clr.b	sensor_status	;
			mov.b	#0x02, r11		; initialize r11 to point to P2.1
			mov.b	#0x00, R12		; initialize R12 to the base of meas_base
CheckNextSensor
			cmp	meas_latest(R12), meas_base(R12)	;
			jl	NotThisSensor		;
			bis.b	r11, sensor_status	; Update sensor_status
NotThisSensor
			rla.b	r11			; Prepare next x
			add.b	#0x02, R12		; Prepare the next index into the array
			cmp.b	#0x40, r11		; Check if done with all five sensors
			jne		CheckNextSensor	;
			ret						;
;-------------------------------------------------------------------------------
; Display Routine:  To be filled in by you.  Turn on the LED that corresponds
; to the 1 bit positions in sensor_status. LED should be ON while button is pressed.
; This routine should NOT use meas_latest or meas_base. It should only use
; sensor_status.
;-------------------------------------------------------------------------------
display:	;check if value for each sensor is below the threshold. if it is turn on that LED.
			mov #8, R12
			cmp meas_base(R12), meas_latest(R12)
			jl dis_Center
			mov #2, R12
			;bic.b	#0xff,&P1OUT	; turn out all LEDs
			cmp meas_base(R12), meas_latest(R12)
			jl dis_down
			mov #6, R12
			cmp meas_base(R12), meas_latest(R12)
			jl dis_up
			mov #0, R12
			;bic.b	#0xff,&P1OUT	; turn out all LEDs
			cmp meas_base(R12), meas_latest(R12)
			jl dis_Left
			mov #4, R12
			cmp meas_base(R12), meas_latest(R12)
			jl dis_Right
			ret

dis_up
			mov.b #1, sensVal ; 1 to up down
			ret
dis_down
			mov.b #2, sensVal; 2 to up down
			ret
dis_Left
			mov.b #3, sensVal; 1 to leftRight
			ret
dis_Right
			mov.b #4, sensVal; 2 to left Right
			ret
dis_Center
			mov.b #0xf, sensVal
			ret

;-------------------------------------------------------------------------------
; Setting up P2.x and TA for the next measurement routine
;-------------------------------------------------------------------------------
; Setting up P2.x to pin oscillation mode
;-------------------------------------------------------------------------------
meas_setup:	bic.b r11,&P2DIR 		; P2.x input
				bic.b r11,&P2SEL 	;
				bis.b r11,&P2SEL2	;
;-------------------------------------------------------------------------------
; The oscillation from P2.x is driving INCLK input of TA0
; No division of this clock source
;-------------------------------------------------------------------------------
		 	mov #TASSEL_3, &TA0CTL 	;
;-------------------------------------------------------------------------------
; Setting up to capture the value of TAR on either rising or falling edges
; using SW based trigger
;-------------------------------------------------------------------------------
			mov #CM_3 + CCIS_2 + CAP, &TA0CCTL1 	;
			ret						;
;-------------------------------------------------------------------------------
; SW delay routine
;-------------------------------------------------------------------------------
SWtimer:	mov	#SWdelay, R14		; Load delay value in r11
Reloadr7	mov	#SWdelay, R13		; Load delay value in R12
ISr70		dec	R13					; Keep this PW for some time*********************************************8
			jnz	ISr70				; The total SW delay count is
			dec	R14					;  = SWdelay * SWdelay
			jnz	Reloadr7			;
			ret						; Return from this subroutine
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

;------------------------------------------------------------------------------------------------------------------------------------------------------
getSamples:
			clr.b &ADC_SW_FLAG 	; Clear ADC SW flag
			clr.b &TAIE_SW_FLAG 	; Clear TAIE SW flag
			clr.w r11 		; Register used to store and process the sample value
			clr.w R13		;register for setting up the array
							; after its acquisition
;-------------------------------------------------------------------------------
; Select analog function on P1.0, i.e. pin2 of the 20PDIP package
;-------------------------------------------------------------------------------
			bis.b #0x01, &ADC10AE0	; P1.0 on pin 2 analog function enabled
;-------------------------------------------------------------------------------
			bis #TASSEL_2 + MC_1+TACLR+TAIE, &TA1CTL ;up mode, divide by one, clear, enable interupts
			bic #ID_3, &TA1CTL
			bis #CCIE, TA1CCTL0 ; enable interupts

			cmp.b #1, upDown;
			jeq up_GS
			cmp.b #2, upDown;
			jeq down_GS

up_GS
			mov #0x2710, TA1CCR0 ;set at 10,000
			;bis #0x2710, TA1CCR0 ;set at 10,000
			jmp test_GS
down_GS
			mov #0x1388, TA1CCR0;set at 5,000
			;bis #0x1388, TA1CCR0;set at 5,000
			jmp test_GS
;-------------------------------------------------------------------------------
; Main loop from here
;-------------------------------------------------------------------------------
test_GS			jmp Reset_test_values
Mainloop_GS
			call #ACQUIRE 	;
			cmp #40, r11 ;if the array is full change r11 to 0
			jeq sendback
			jmp	Mainloop_GS	;


;-------------------------------------------------------------------------------
ACQUIRE:
			clr.w	&ADC10CTL0	; Clear configuration registers just in case
			clr.w	&ADC10CTL1	; some values were left on by a prior routine
;-------------------------------------------------------------------------------
; ADC10CTL0 configuration based on the CLR instruction above and the one below:
;	SREF=001, ADC10SHT=64*ADC10CLKs, ADC10SR=0, REFOUT=0, REFBURST=0, MSC=0,
;	REF2_5=0, REFON=1, ADC10ON=1, ADC10IE=1, ADC10IFG=0, ENC=0, ADC10SC=0
;-------------------------------------------------------------------------------
			bis.w #(SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE), &ADC10CTL0 ;
;-------------------------------------------------------------------------------
; ADC10CTL1 configuration based on the CLR instruction above and the ones below:
;	INCH=1010, SHS=00, ADC10DF=0, ISSH=0, ADC10DIV=/8, ADC10SSEL=00,
;	CONSEQ=00, ADC10BUSY=0
;-------------------------------------------------------------------------------
			;bis.w #(INCH_10 + ADC10DIV_7), &ADC10CTL1 ; Input channel = int. temp. diode
			bis.w #(INCH_0 + ADC10DIV_7), &ADC10CTL1 ; Input channel = P1.0
			eint 				; Enable general interrupts
			clrz 				; Clear Z
			bis #CCIE, TA1CCTL0 ; enable interupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
;-------------------------------------------------------------------------------
CheckFlag_C	tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was
			jz CheckFlag_C 		; executed
			clr.b &TAIE_SW_FLAG
;-------------------------------------------------------------------------------
			bic #CCIE, TA1CCTL0 ; disable interupts
			bic #MC0+TACLR, &TA1CTL ; starts timer b
			clr.b &ADC_SW_FLAG 		; Clear ADC SW FLAG
			bis.w #(ENC + ADC10SC), &ADC10CTL0 ; Start a conversion     ****ADC10SC starts the adc process
CheckFlag	tst.b &ADC_SW_FLAG 		; Check to see if ADC10_ISR was
			jz CheckFlag 		; executed
			dint				; Disable general interrupts
			clr.w	&ADC10CTL0	; Clear configuration registers
			clr.w	&ADC10CTL1	; Safe practice
			;cmp #64, r11 ;if the array is full change r11 to 0
			;jeq quick_end
			;mov #0, r11
			ret					;
;-------------------------------------------------------------------------------
Reset_test_values:
					mov	#0x1010, Array(R13)
					add #2, R13;
					cmp #40, R13
					jne Reset_test_values
					jmp Mainloop_GS

quick_end
;			dint
			nop
			jmp quick_end
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
convertSamples:
			clr R13
			cmp.b #0x3, leftRight
			jeq Linear
			cmp.b #0x4, leftRight
			jeq Log_CS

Linear:
			rra Array(R13) ; 10bits -> 9
			rra Array(R13) ; 9bits -> 8
			bic #0xFF00, Array(R13) ; clear bits 8 - 15
			cmp #0x20, Array(R13) ; 32
			jl	mov_1;
			cmp #0x40, Array(R13) ; 64
			jl	mov_2;
			cmp #0x60, Array(R13) ; 96
			jl	mov_3;
			cmp #0x80, Array(R13) ; 128
			jl	mov_4;
			cmp #0xa0, Array(R13) ; 160
			jl	mov_5;
			cmp #0xc0, Array(R13) ; 192
			jl	mov_6;
			cmp #0xe0, Array(R13) ; 224
			jl	mov_7;
			jmp mov_8			;greater than or equal to 224 (1110 0000)
next_el_Lin
			add #2, R13;
			cmp #40, R13
			jne Linear
			jmp sendback

Log_CS:
			rra Array(R13) ; 10bits -> 9
			rra Array(R13) ; 9bits -> 8
			bic #0xFF00, Array(R13) ; clear bits 8 - 15
			cmp #0x1, Array(R13)  ; < 0000 0001
			jl	mov_0;
			cmp #0x2, Array(R13) ; < 0000 0010
			jl	mov_1;
			cmp #0x4, Array(R13) ; < 0000 0100
			jl	mov_2;
			cmp #0x8, Array(R13) ; < 0000 1000
			jl	mov_3;
			cmp #0x10, Array(R13) ; < 0001 0000
			jl	mov_4;
			cmp #0x20, Array(R13) ; < 0010 0000
			jl	mov_5;
			cmp #0x40, Array(R13) ; < 0100 0000
			jl	mov_6;
			cmp #0x80, Array(R13) ; < 1000 0000
			jl	mov_7;
			jmp mov_8			;>= 1000 0000
next_el_Log
			add #2, R13;
			cmp #40, R13
			jne Log_CS
			jmp sendback
;-------------------------------------------------------------------------------
mov_0
			mov #0x0 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_1
			mov #0x1 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_2
			mov #0x2 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_3
			mov #0x3 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_4
			mov #0x4 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_5
			mov #0x5 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_6
			mov #0x6 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_7
			mov #0x7 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
mov_8
			mov #0x8 ,LED_Array(R13)
			cmp.b #0x3, leftRight ; go back to the corrent path. return was being buggy
			jeq next_el_Lin
			jmp next_el_Log
;---------------------------------------------------------------------------------------------------------------------------------------------------------------------
displaySamples:
;-------------------------------------------------------------------------------
; Setup P1.2 as TXD and P1.1 as RXD; see data sheet port schematic and UG
;-------------------------------------------------------------------------------
SetupP1 	bis.b #6h,&P1SEL 	; P1.2/P1.1 = USART0 TXD/RXD
			bis.b #6h,&P1SEL2 	; P1.2/P1.1 = USART0 TXD/RXD
			clr R14 ; keeps value
;-------------------------------------------------------------------------------
; Setup USI in UART mode, minimal configuration, point to point
;-------------------------------------------------------------------------------
SetupUART0 	clr.b &UCA0CTL0 		; default values - see UG
			clr.b &UCA0CTL1 		; default values - see UG
			bis.b #UCSSEL1 + UCSSEL0,&UCA0CTL1 ; UCLK = SMCLK ~1 MHz
			clr.b &UCA0STAT 		; default values - see UG
		;	bis.b #UCLISTEN,&UCA0STAT ; loopback - used for debugging only
;-------------------------------------------------------------------------------
; For a baud rate of 9600,the pre-scaler value is
;    = (UCAxBR0 + UCAxBR1 � 256) = 104 in decimal - integer part - see UG
;-------------------------------------------------------------------------------
			mov.b #069h,&UCA0BR0 	; Baud Rate = ? - YOU MUST COME UP WITH THIS VALUE  USED 69
			mov.b #000h,&UCA0BR1 	; UCBRx = ?		- FOR THE REQUIRED BAUD RATE
;-------------------------------------------------------------------------------
; Modulation Control Register - fractional part - see UG
;-------------------------------------------------------------------------------
			mov.b #002h,&UCA0MCTL 	; UCBRFx = 0, UCBRSx = 1, UCOS16 = 0
;-------------------------------------------------------------------------------
; SW reset of the USI state machine
;-------------------------------------------------------------------------------
			bic.b #UCSWRST,&UCA0CTL1 ; **Initialize USI state machine**
			bis.b #UCA0RXIE,&IE2 	; Enable USART0 RX interrupt
	 		bis.b #GIE,SR 			; General Interrupts Enabled
;-------------------------------------------------------------------------------
; After the state machine is reset, the TXD line seems to oscillate a few times
; It is therefore safer to check if the machine is in a state in which it is
; ready to transmit the next byte.  Don't remove this code!
;-------------------------------------------------------------------------------
TX2
			eint
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz TX2 					; Jump if TX buffer not ready
			mov.b #0x55,&UCA0TXBUF 	; TX <U> charac. eq. to #0x55 in ASCII

;-------------------------------------------------------------------------------
; Always check if the transmit buffer is empty before loading a new value in!
; We send a dummy "Hi" to make sure the interface is working.
;-------------------------------------------------------------------------------
			call #char_ret;
			;jmp char_ret
;-------------------------------------------------------------------------------
; Mainloop starts here:
;-------------------------------------------------------------------------------

			eint
;-------------------------------------------------------------------------------
			bis #TASSEL_2 + ID_3 + MC_1+TACLR, &TA1CTL ;smclock, /8  upmode, clear
			bis #CCIE, TA1CCTL0 ; enable interupts
			bis #0xF424, TA1CCR0 ;set at 500,000 / 8
reset_DS	clr R10
Mainloop_DS
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
;-------------------------------------------------------------------------------
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			cmp #0x0 ,LED_Array(R10)
			jeq dis_0_DS
			cmp #0x1 ,LED_Array(R10)
			jeq dis_1_DS
			cmp #0x2 ,LED_Array(R10)
			jeq dis_2_DS
			cmp #0x3 ,LED_Array(R10)
			jeq dis_3_DS
			cmp #0x4 ,LED_Array(R10)
			jeq dis_4_DS
			cmp #0x5 ,LED_Array(R10)
			jeq dis_5_DS
			cmp #0x6 ,LED_Array(R10)
			jeq dis_6_DS
			cmp #0x7 ,LED_Array(R10)
			jeq dis_7_DS
			cmp #0x8 ,LED_Array(R10)
			jeq dis_8_DS
next_el_DS
			add #2, R10;
			cmp #40, R10
			jne Mainloop_DS
			jmp reset_DS
			;
;------------------------------------------------------------------------------
char_ret
TXD			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz TXD 					; Jump if TX buffer not ready
			mov.b #0x0D,&UCA0TXBUF 	; TX <carriage return> charac. eq. to #0x0D in ASCII
TXA			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz TXA 					; Jump if TX buffer not ready
			mov.b #0x0A,&UCA0TXBUF 	; TX <line feed> charac. eq. to #0x0A in ASCII
			ret ; =================================================================================how do I get this to return?
;------------------------------------------------------------------------------
CheckFlag_C_DS:
			eint 				; Enable general interrupts
			clrz 				; Clear Z
			tst.b &TAIE_SW_FLAG_2 		; Check to see if TAIE_ISR was
			jz CheckFlag_C_DS 		; executed
			clr.b &TAIE_SW_FLAG
		;	dint				; Disable general interrupts
			ret

SWtimer_DS:
			mov	#0xF, R14		; Load delay value in r11
Reloadr7_DS	mov	#0XF, R13		; Load delay value in R12
ISr70_DS	dec	R13					; Keep this PW for some time*********************************************8
			jnz	ISr70_DS				; The total SW delay count is
			dec	R14					;  = SWdelay * SWdelay
			jnz	Reloadr7_DS			;
			ret						; Return from this subroutine
;------------------------------------------------------------------------------
center_DS:
			;---------------------
			call #meas_latest_val		;
			call #det_sensor		;
			;---------------------
			;---------------------
			call #display			;
			;cmp.b	#0x0f, sensVal
			;jeq	sendback
			;---------------------
			ret
;------------------------------------------------------------------------------
dis_0_DS:
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_0_DS 				; Jump if TX buffer not ready
			mov.b #0x30,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_0:

			bic.b	#0xff,&P1OUT	; turn out all LEDs
			;
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_0 		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_1_DS:
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_1_DS 				; Jump if TX buffer not ready
			mov.b #0x31,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_1

			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0x80, &P1OUT	; turn on LED4
			;
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_1		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_2_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_2_DS 				; Jump if TX buffer not ready
			mov.b #0x32,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_2
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xC0, &P1OUT	; turn on LED 4&3
			;
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_2 		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_3_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_3_DS 				; Jump if TX buffer not ready
			mov.b #0x33,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_3
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xE0, &P1OUT	; turn on LED 4, 3, 2
			;
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_3 		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_4_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_4_DS 				; Jump if TX buffer not ready
			mov.b #0x34,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_4


			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xF0, &P1OUT	; turn on LED 4, 3, 2, 1
			;
			call #center_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_4	; executed
			;---------------
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_5_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_5_DS 				; Jump if TX buffer not ready
			mov.b #0x35,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_5

			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xF0, &P1OUT	; turn on LED 4, 3, 2, 1
			;
			call #center_DS
			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x10,&P1OUT	; turn on LED 5
			call	#SWtimer_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;

			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x10,&P1OUT	; turn on LED 5
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_5		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_6_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_6_DS 				; Jump if TX buffer not ready
			mov.b #0x36,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_6

			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xF0, &P1OUT	; turn on LED 4, 3, 2, 1
			;
			call #center_DS

			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x30,&P1OUT	; turn on LED 5, 6
			call	#SWtimer_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x30,&P1OUT	; turn on LED 5, 6
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_6 		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_7_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_7_DS 				; Jump if TX buffer not ready
			mov.b #0x37,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_7

			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xF0, &P1OUT	; turn on LED 4, 3, 2, 1
			;
			call #center_DS

			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0x70,&P1OUT	; turn on LED 5, 6,7
			call	#SWtimer_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_7		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
dis_8_DS
			bit.b #UCA0TXIFG,&IFG2 	; USI TX buffer ready?
			jz dis_8_DS 				; Jump if TX buffer not ready
			mov.b #0x38,&UCA0TXBUF	;number of leds in askii
			call #char_ret

			eint  				; Enable general interrupts
			bis #MC0+TACLR, &TA1CTL ; starts timer b
			clrz 				; Clear Z
dis_flag_8

			bic.b	#0xf8,&P1OUT	; prepare to display LEDs 1-4
			bis.b	#0xF0, &P1OUT	; turn on LED 4, 3, 2, 1
			;
			call #center_DS

			bic.b	#0xf8,&P1OUT	; turn out all LEDs
			bis.b	#0xf8,&P1OUT	; prepare to display LEDs 5-8
			bic.b	#0xF0,&P1OUT	; turn on LED 5, 6, 7, 8
			call	#SWtimer_DS
			cmp.b	#0x0f, sensVal
			jeq	sendback
			;
			tst.b &TAIE_SW_FLAG 		; Check to see if TAIE_ISR was executed
			jz dis_flag_8 		; executed
			clr.b &TAIE_SW_FLAG
			bic #MC0+MC1, &TA1CTL ; stops timer b
			dint 				; Disable general interrupts
			bic.b	#0xff,&P1OUT	; turn out all LEDs
			jmp next_el_DS
;------------------------------------------------------------------------------
sendback   ret

;-------------------------------------------------------------------------------
; Interrupt Service Routines
;-------------------------------------------------------------------------------
ADC10_ISR:
			nop					;
			bic.w #ADC10IFG, &ADC10CTL0	;
			mov.w &ADC10MEM, R15
			;rra R15;
			;rra R15;
			;bic #0xFF00, R15
			mov.w R15, Array(r11)	;
			add #2, r11;
;-------------------------------------------------------------------------------
; Set the ADC SW flag, which is continuously checked by the ACQUIRE routine
;-------------------------------------------------------------------------------
			mov.b #0x01, &ADC_SW_FLAG ;
return
			reti 				;
;-------------------------------------------------------------------------------
TAIFG_ISR:

			;clear TAIFG flag or whatever causes the interrupt
			nop
			bic #TAIFG, &TA0CTL
			bic #CCIFG, &TA0CTL
			bic #TAIFG, &TA1CTL
			bic #CCIFG, &TA1CTL
;-------------------------------------------------------------------------------
			mov.b #0x01, &TAIE_SW_FLAG ;
			reti 				;

;-------------------------------------------------------------------------------
Back_ISR_0:
			nop;
			bic #TAIFG, &TA1CTL
			reti;
Back_ISR_1:
			nop;
			reti;
;------------------------------------------------------------------------------
; Echo back RXed character, confirm TX buffer is ready first
;------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
;           Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect 	.stack
;-------------------------------------------------------------------------------
;           Interrupt Vectors
;-------------------------------------------------------------------------------
  ;          .sect   ".reset"		; MSP430 RESET Vector
   ;         .short  RESET
;-------------------------------------------------------------------------------
			.sect ".int05" 			; ADC1	0 Vector
isr_adc10: 	.short ADC10_ISR ;

			.sect ".int13"
ISR_1:		.short TAIFG_ISR
