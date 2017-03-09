;
; This template provides working code to drive the speaker and two 7-segment LEDs
;
; Template subroutines are provided to read button inputs, configure interrupts (Lab #4) and drive the 8 LEDs on the ECE Tiva Shield
;
; In order to get the code to assemble you will have to fix lines which have had critical numbers removed - or comment them out if they're not needed yet

;
; Port C init and use kills programming the CPU!
; Port D bits 0 and 1 are disabled in mask as they're shorted with port B bits 6 and 7 on the Tiva-C
;
;

; This code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
; The code provided initializes all 3 ports (A,B,E) with the ECE LED bar array plugged into the Tiva board
; Port F with the Tiva 3 LEDs (Red, Green, Blue) and two buttons is also initialized
; Then the LEDs on each port are turned off and on with time delays - while the Tiva board R, G, B LEDs are turned on and off

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

; These equates allow one to associate a name with a value to make the code more readable

RED       EQU 0x02		; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10          ; on the left side of the Tiva board
SW2       EQU 0x01          ; on the right side of the Tiva board

Start
	BL  Port_Init			; initialize input and output pins of Ports A to F
	BL	Interrupt_Init		; Init interrupts for the switches on the Tiva for Lab #4



Start_Of_Demo
	MOV R5, #0		; display starting from 0
	MOV R4, #0x2710	; this delay should be random number
    MOV	R11, #0xABCD	
    BL RandomNum
	BL Delay		; delay R1 x base delay	
	MOV R0,#2_00000010
	BL Display_LED
    BL POLL
DisplayLoop		
				MOV R5, R9		; Display the first 8-bit 
				BL Display_Number
				
			;	MOV R0,R9
			;	BL Display_LED
				
				LSR R5, #8
				MOV R4, #0x4E20
				BL Delay	; Delay for 2 seconds
				BL Display_Number ; Display the next 8 bits
				
;				LSR R0, #8
			;	BL Display_LED
				
				LSR R5, #8
				MOV R4, #0x4E20
				BL Delay	; Delay for 2 seconds
				BL Display_Number  ; Display the next 8 bits
				
		;		LSR R0, #8
		;		BL Display_LED
				
				LSR R5, #8
				MOV R4, #0x4E20
				BL Delay	; Delay for 2 seconds
				BL Display_Number  ; Display the final 8 bits
				
			;	LSR R0, #8
			;	BL Display_LED
				
				MOV R4, #0xC350
				BL Delay	; Delay for 5 seconds 
				B DisplayLoop

	
SOMEDELAY             EQU 400      ; faction of a second delay
POLL			STMFD		R13!,{R1, R2, R3, R14}
		;
		; code to generate a delay of 0.1mS * R0 times
		;
				MOV R9, #1
									
MultipleLoop2	
				LDR	R2,=SOMEDELAY
loopMore2		
				SUBS	R2, #1			
				BNE	loopMore2
				ADD R9, R9, #1	; increments one every 0.1 millisecond
				;AND R9, #0xff
				BL Check_Buttons		; return non zero Z status upon button press
	            BEQ MultipleLoop2	; keep going if there is no button press
				
			
exitDelay		LDMFD		R13!,{R1, R2, R3, R15}
; ----------Display_LED----------------
; Display the 8-bit number on the 8 LEDs
; Input: R0
; Output: none
Display_LED
	STMFD		R13!,{R2, R14}		; push the LR or return address

;Input data is in R0 - the lowest 8 bits
;Use Appendix H in the lab manual to write the correct bits to port B and E to drive all 8 LEDs
;This is much easier than the Keil board, or the old Tiva shield, as you'll notice that 7 of the 8 LEDs on the ECE Tiva Shield are in order on one port

	LDR R2, =GPIO_PORTB + (PORT_B_MASK << 2)
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port B where the LEDs are

	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port B where the LEDs are

	LDMFD		R13!,{R2, R15}		; pull the LR or return address and return


;------------Display_Number------------
; Display a 8-bit number on the two 7-segment digits
; Output: none
; Input: R5, 8 LSB get displayed
;
; Internal use: R1 holds Port A, R2 holds Port B, R3 holds Port E values to be written

Display_Number
	STMFD		R13!,{R0-R7, R9, R14}		; push the LR or return address

; First take the 4 LSB and display that to HEX #2

	PUSH {R5}	; keep a copy of the original number

	LDR R9, =Seven_Seg_Table
	MOV R7, R5		; keep a copy of the original number in R7
	AND R5, #0x0f
	LSL R0, R5, #1	; multiply by two as the lookup table holds 16-bit data
	LDRH R4, [R9, R0]		; table base + offset * 2
	EOR R4, #0xfe		; active low 7-segment so invert the bits going out and remove the DP on bit 0
	; DP is Bit 0, then A, B, C, D, E, F, G (bit 7)
	
; Display least byte on Seven_Segment_2	
	MOV R6, #0	; R5 is the output to port C, segment C
	MOV R9, R4
	LSL R4, #1		; port A bits 0 and 1 are unused, data already has DP in LSB
	
	ANDS R0, R4, #0x10	; test segment C
	ORRNE R6, #0x4		; segment C is bit 4 on port C - CHANGED to E2
	
	AND R4, #0xEF	; clear bit 4
	ANDS R0, R9, #0x80	; segment G goes into bit 4
	ORRNE R4, #0x10	; set bit 4 if G segment is a 1
	LDR R2, =GPIO_PORTA + (PORT_A_MASK << 2)
	STR R4, [R2, #GPIO_DATA_OFFSET]		; write output to port A, A segment is big 2, bit 4 is G, C goes elsewhere
	
	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R0, [R2, #GPIO_DATA_OFFSET]		; read, insert bit 4 and write back port E2
	AND R0, #0xFB
	ORR R0, R0, R6
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write segment C to port E2
	
	;MOV R4, #0x2710
	;BL Delay

; Now handle 7-segment #1 with the 4 LSB
	POP {R5}
	LSR R5, #4		; shift down the upper nibble
	AND R5, #0x0f	; keep only the 4 bits to display
	LSL R0, R5, #1	; multiply by two as the lookup table holds 16-bit data
	LDR R9, =Seven_Seg_Table
	LDRH R4, [R9, R0]		; table base + offset * 2
	EOR R4, #0xfe		; active low 7-segment so invert the bits going out and remove the DP on bit 0

	MOV R0, R4	; build port D output in R0
	LSR R0, #1	; push off DP
	MOV R1, R0	; copy the bit pattern and keep lowest 2 bits for port F bits 2, 3
	AND R1, #0x3	; keep only 2 LSBs and shift them up 2 bits into F2, F3
	LSL R1, #2
	LDR R2, =GPIO_PORTF + (PORT_F_MASK << 2)
	STR R1, [R2, #GPIO_DATA_OFFSET]		; write out Segment 1 A, B into F2, F3
		
	AND R0, #0x0f	; clear all but lower nibble as they're direct out
	; build into R1 the upper 2 bits and or it back in
	ANDS R1, R4, #0x40	; segment F
	ORRNE R0, #0x80
	ANDS R1, R4, #0x20	; segment E
	ORRNE R0, #0x40
	LDR R2, =GPIO_PORTD + (PORT_D_MASK << 2)
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port D four of the 7-segment #1 bits
	
	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R0, [R2, #GPIO_DATA_OFFSET]		; read-modify-write output to port D most of 7-segment #1 bits
	AND R0, #0xFC	; zero all but two LSB as those are what we're updating
	
	ANDS R1, R4, #0x80		; test segment G
	ORRNE R0, #0x1			; insert segment G into LSB
	ANDS R1, R4, #0x1		; test segment DP
	ORRNE R0, #0x2			; insert segment DP into bit 1
	STR R0, [R2, #GPIO_DATA_OFFSET]		; write output to port D most of 7-segment #1 bits
	
	LDMFD		R13!,{R0-R7, R9, R15}		; pull the LR or return address and return

; Check Tiva Buttons subroutine
; R0 returns 0 if no button pressed and non-zero upon a button press (SW1 and SW2 on the Tiva board)
;
Check_Buttons
	STMFD		R13!,{R1, R14}		; push the LR or return address
	LDR R1, =GPIO_PORTF_DATA_R ; pointer to Port F data register
	LDR R0, [R1]		; Read in the current Port F inputs
	AND R0, #1		; return only the bits that have a button
	EORS R0, #1		; bits go to 0 if button is pressed - so invert them
	LDMFD		R13!,{R1, R15}		; pull the LR or return address and return

; Beep the speaker on the ECE Shield using port E4 and E5
; The speaker is conencted to two pins - toggle each end for more volume than a singled ended drive
; Ensure that each beep is about the same length - 0x300 loops of delay loop
; Input: R1 sets the tone - 2 is a high pitch
; Output: none
;------------RandomNum------------
; R11 holds a 16-bit random number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; R11 holds a non-zero 16-bit number.  If a zero is fed in, as the seed, the pseudo-random sequence will stay stuck at 0
; Take as many bits of R11 as you need.  If you take the lowest 4 bits then you get a number between 1 and 15.
;   If you take bits 5..1 you'll get a number between 0 and 15 (assuming you right shift by 1 bit).
;
; R11 MUST be initialized to a non-zero 16-bit value at the start of the program OR ELSE!
; R11 can be read anywhere in the code but must only be written to by this subroutine
;
RandomNum		STMFD		R13!,{R1, R2, R3, R14}

				AND			R1, R11, #0x8000
				AND			R2, R11, #0x2000
				LSL			R2, #2
				EOR			R3, R1, R2
				AND			R1, R11, #0x1000
				LSL			R1, #3
				EOR			R3, R3, R1
				AND			R1, R11, #0x0400
				LSL			R1, #5
				EOR			R3, R3, R1		; the new bit to go into the LSB is present
				LSR			R3, #15
				LSL			R11, #1
				ORR			R11, R11, R3
				
				LDMFD		R13!,{R1, R2, R3, R15}
;------------Poll------------
;SOMEDELAY             EQU 4000000     ; 0.1ms delay


;------------wtf------------

;	BX LR
;------------DELAY------------

				


Delay			STMFD		R13!,{R0,R2, R14}
		;
		; code to generate a delay of 0.1mS * R0 times
		;
				
				MOV R0, R4				
MultipleLoop	
                LDR	R2, =SOMEDELAY  
loopMore		
                SUBS	R2, #1			
				BNE	loopMore
				SUBS R0, #1;
				BNE MultipleLoop
			
				LDMFD		R13!,{R0,R2, R15}


; Registers have one bit (starting at LSB - same as other I/O such as LEDS) for each I/O pin

GPIO_IS_OFFSET   EQU 0x404	;GPIOIS - Interrupt Sense : 0 = edge, 1 = level interrupt
GPIO_IBE_OFFSET  EQU 0x408	;GPIOIBE - Interrupt Both Edges : 0 = single edge, 1 = both edges
GPIO_IEV_OFFSET  EQU 0x40c	;GPIOIEV - Interrupt Event : 0 = low level or falling edge, 1= high level or rising edge
GPIO_IM_OFFSET   EQU 0x410	;GPIOIM - Interrupt Mask : 0 = masked, 1 = unmasked
GPIO_RIS_OFFSET  EQU 0x414		; Raw Interrupt Status - READ ONLY
GPIO_MIS_OFFSET  EQU 0x418		; Masked Interrupt Status - READ ONLY
GPIO_ICR_OFFSET  EQU 0x41c		; Interrupt Clear - writing a 1 clears the RIS and MIS registers

Interrupt_Init
	STMFD		R13!,{R14}		; push the LR or return address

;Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
;and mask of the interrupts for each port.
;Note: To prevent false interrupts, the following steps should be taken when re-configuring
;GPIO edge and interrupt sense registers:

;a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
	
	LDR R1, =GPIO_PORTF
    MOV R0, #0x00             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

; b. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE register.

    MOV R0, #0x00             ; 0 means edge detecting interrupt
    STR R0, [R1, #GPIO_IS_OFFSET]	; 

    MOV R0, #0x00             ; 0 means falling edge detecting
    STR R0, [R1, #GPIO_IEV_OFFSET]	; 

    MOV R0, #0x00             ; 0 means single edge detection
    STR R0, [R1, #GPIO_IBE_OFFSET]	; 

;c. Clear the GPIORIS register using the ICR register to clear any pending interrupts.
; The switches are bits 0 and 4
    MOV R0, #0x80             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_ICR_OFFSET]	; clear any interrupts recieved

;d. Unmask the port by setting the IME field in the GPIOIM register.
; Set the appropiate bit to 1 to enable interrupts for only the one switch required
    MOV R0, #0x80           ; 0 means mask or block interrupts

	STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

;Looking in the Startup.s file one will find an EXPORT of the address for interrupt handlers, one for each GPIO port

; Interrupt Enable Registers
CORE_PERIPHERALS 		EQU 0xe000e000
INTERRUPT_EN0_OFFSET 	EQU 0x100

; The Interrupt Number (Bit in Interrupt Registers) value written to the EN0 register to enable port F interrupts can be found in Table 2-9 (page 104)
	LDR R0,=SOMEDELAY;# 0x????????	; this 32-bit value enables GPIO Port F Interrupts via a single bit set

	LDR R1, =CORE_PERIPHERALS
	STR R0, [R1, #INTERRUPT_EN0_OFFSET]		; GPIO Interrupts require this enable

	LDMFD		R13!,{R15}		; push the LR or return address
	
;Section 3.1.2 Nested Vector Interrupt Controller

;The Cortex-M4F processor supports interrupts and system exceptions. The processor and the
;Nested Vectored Interrupt Controller (NVIC) prioritize and handle all exceptions. An exception
;changes the normal flow of software control. The processor uses Handler mode to handle all
;exceptions except for reset. See “Exception Entry and Return” on page 108 for more information.
;The NVIC registers control interrupt handling. See “Nested Vectored Interrupt Controller
;(NVIC)” on page 124 for more information.

;Table 3-8 on page 134 details interrupt Set / Clear 
; they allow one to enable individual interrupts and DIS? lets one disable individual interrupt numbers

; Table 2-9 Interrupts on page 104 details interrupt number / bit assignments
; Port F - Bit 30
; Timer 0A Bit 19
; Timer 0B Bit 20
 
;For edge-triggered interrupts, software must clear the interrupt to enable any further interrupts.


; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.

GPIO_PORTF_DIR_R   EQU 0x40025400		; Port F Data Direction Register setting pins as input or output
GPIO_PORTF_DATA_R  EQU 0x400253FC		; address for reading button inputs
	
; These are the configuration registers which should not be touched
; Port Base addresses for the legacy (not high-performance) interface to I/O ports
GPIO_PORTA			EQU 0x40004000
GPIO_PORTB			EQU 0x40005000
GPIO_PORTC			EQU 0x40006000
GPIO_PORTD			EQU 0x40007000
GPIO_PORTE			EQU 0x40024000
GPIO_PORTF			EQU 0x40025000

; These are the masks for pins which are outputs
PORT_A_MASK			EQU 0xfc	;0xE0		; PA7,6,5 are outputs for LEDs
PORT_B_MASK			EQU 0xff	;3f	; exclude B2:3 0xff	;33		; PB5,4,1,0 are outputs %0011 0011 
PORT_C_MASK			EQU 0x30	; this hangs the CPU 0xf0	
PORT_D_MASK			EQU 0xcc	;exclude d7 0xcf	Disable D0, D1 due to short with B6, B7
PORT_E_MASK			EQU 0x3f	;0x30		; PE5,4 are outputs %0011 0000
PORT_F_MASK			EQU 0x0e		; PF has LEDs on PF 1,2,3 and buttons PF0, PF4 (don't enable buttons as outputs)
	
; Offsets are from table 10-6 on page 660
GPIO_DATA_OFFSET	EQU 0x000		; Data address is the base address - YOU HAVE TO ADD AN ADDRESS MASK TOO to read or write this!!
GPIO_DIR_OFFSET		EQU 0x400		; Direction register
GPIO_AFSEL_OFFSET EQU 0x420			; Alternate Function SELection
GPIO_PUR_OFFSET   EQU 0x510			; Pull Up Resistors
GPIO_DEN_OFFSET   EQU 0x51C			; Digital ENable
GPIO_LOCK_OFFSET  EQU 0x520
GPIO_CR_OFFSET    EQU 0x524
GPIO_AMSEL_OFFSET EQU 0x528			; Analog Mode SELect
GPIO_PCTL_OFFSET  EQU 0x52C

SYSCTL_HBCTL  EQU   0x400FE06C		; high performance bus control for ports A to F

GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
SYSCTL_RCGCGPIO_R  EQU   0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2, R3
Port_Init
	STMFD		R13!,{R14}		; push the LR or return address

; First enable the clock to the I/O ports, by default the clocks are off to save power
; If a clock is not enabled to a port and you access it - then the processor hard faults
	LDR R1, =SYSCTL_RCGCGPIO_R      ; activate clock for Ports (see page 340)
    LDR R0, [R1]                 
    ORR R0, R0, #0x3F               ; turn on clock to all 6 ports (A to F, bits 0 to 5)
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
	
; Set all ports to APB bus instead of AHB - this should be unnecessary
;	LDR R1, =SYSCTL_HBCTL
;	LDR R0, [R1]
;	AND R0, #0xFFFFFFE0		; set Ports A thru F to APB (0) and leave the rest at their default
;	STR R0, [R1]

; Page 650, Table 10-1 GPIO Pins with Special Considerations.
; These pins must be left as configured after reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Initialize the I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; Call Port_Init_Individual with the following paramaters passed:
; R1 is the base port address
; R2 is the output pin mask (which bits are outputs)
; R3 is the input pin mask  (which bits get configured as inputs)

	MOV R3, #0x00				; Select no pins as input (unless it's changed as for port F)
	
; Init Port A, B, E are by default GPIO - set all output pins used to a 1 to enable them
;   and leave all of the other pins as previously configured!
    LDR R1, =GPIO_PORTA
    MOV R2, #PORT_A_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port B
    LDR R1, =GPIO_PORTB
    MOV R2, #PORT_B_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port C
    LDR R1, =GPIO_PORTC
    MOV R2, #PORT_C_MASK
	;BL Port_Init_Individual

; Init Port D
    LDR R1, =GPIO_PORTD
    MOV R2, #PORT_D_MASK
	BL Port_Init_Individual

; Init Port E
	LDR R1, =GPIO_PORTE
    MOV R2, #PORT_E_MASK			; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port F
	LDR R1, =GPIO_PORTF
    MOV R2, #PORT_F_MASK		; enable commit for Port, 1 means allow access
	MOV R3, #0x11				; enable weak pull-up on PF0 and PF4 (buttons)
	BL Port_Init_Individual

	LDMFD		R13!,{R15}		; pull the LR or return address from the stack and return

;----------------------------------------------------------

;-----------------------------------------------------------
;------------Port_Init_Individual------------
; Initialize one GPIO Port with select bits as inputs or outputs
; Output: none
; Input: R1, R2, R3
; R1 has to be the port address
; R2 has to hold the mask for output pins
; R3 has to be the mask for input pins
; Modifies: R0

Port_Init_Individual
	STMFD		R13!,{R14}		; push the LR or return address
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1, #GPIO_LOCK_OFFSET]	; 2) unlock the lock register
	ORR R0, R2, R3					; all access to inputs and outputs as masked in R2 and R3
    STR R0, [R1, #GPIO_CR_OFFSET]	; enable commit for Port F
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1, #GPIO_AMSEL_OFFSET]	; 3) disable analog functionality
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1, #GPIO_PCTL_OFFSET]	; 4) configure as GPIO
    LDR R0, [R1, #GPIO_DIR_OFFSET]	; 5) read default direction register configuration
    ORR R0, R2						; ORR in only the bits we want as outputs
    STR R0, [R1, #GPIO_DIR_OFFSET]	; 5) set direction register
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1, #GPIO_AFSEL_OFFSET]	; 6) regular port function
    STR R3, [R1, #GPIO_PUR_OFFSET]	; pull-up resistors for PF4,PF0
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1, #GPIO_DEN_OFFSET]
	LDMFD		R13!,{R15}		; pull the LR or return address and return

	ALIGN
Port_Table
	DCD	GPIO_PORTA + (PORT_A_MASK << 2)		; DCD - Define Constant Double Word (32-bits)
	DCD	GPIO_PORTB + (PORT_B_MASK << 2), GPIO_PORTC + (PORT_C_MASK << 2)
	DCD	GPIO_PORTD + (PORT_D_MASK << 2), GPIO_PORTE + (PORT_E_MASK << 2)
	DCD	GPIO_PORTF + (PORT_F_MASK << 2), 0

	ALIGN
Seven_Seg_Table
	DCW 0x7e, 0x0c, 0xb6, 0x9e, 0xcc, 0xda, 0xfa, 0x0e, 0xfe, 0xce	; 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
	DCW 0xee, 0xf8, 0xb0, 0xbc, 0xf2, 0xe2		; A, B, C, D, E, F with LSB being 0 for the DP
	
    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file