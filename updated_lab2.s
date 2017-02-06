;*----------------------------------------------------------------------------
;* Name:    Lab_2_program.s 
;* Purpose: This code template is for Lab 2 on the Tiva Board
;* Author: Eric Praetzel and Rasoul Keshavarzi 
;*----------------------------------------------------------------------------*/

; This code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
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

; One can also define an address, or constant, using the EQUate directive

GPIO_PORTF_DATA_R  EQU 0x400253FC
GPIO_PORTF_DIR_R   EQU 0x40025400
GPIO_PORTF_AFSEL_R EQU 0x40025420
GPIO_PORTF_PUR_R   EQU 0x40025510
GPIO_PORTF_DEN_R   EQU 0x4002551C
GPIO_PORTF_LOCK_R  EQU 0x40025520
GPIO_PORTF_CR_R    EQU 0x40025524
GPIO_PORTF_AMSEL_R EQU 0x40025528
GPIO_PORTF_PCTL_R  EQU 0x4002552C
GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
RED       EQU 0x02
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; on the left side of the Launchpad board
SW2       EQU 0x01                 ; on the right side of the Launchpad board
SYSCTL_RCGCGPIO_R  EQU   0x400FE608


        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

SOMEDELAY             EQU 3000000      ; approximately 1s delay at ~16 MHz clock
Start
;
; Turn off all LEDs 
    BL  PortF_Init                  ; initialize input and output pins of Port F
	
ResetLUT
		LDR         R5, =InputLUT            ; assign R5 to the address at label LUT
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
NextChar
        	LDRB        R2, [R5]		; Read a character to convert to Morse
        	ADD         R5, #1              ; point to next value for number of delays, jump by 1 byte
	    	TEQ         R2, #0              ; If we hit 0 (null at end of the string) then reset to the start of lookup table
			BNE		    ProcessChar	; If we have a character process it
			BEQ         starNextCycle
			
ProcessChar	BL		CHAR2MORSE	; convert ASCII to Morse pattern in R1		

;	First - loop until we have a 1 bit to send  (no code provided)
;
;	This is confusing as we're shifting a 32-bit value left, but the data is ONLY in the lowest 16 bits, so test starting at bit 15 for 1 or 0
;	Then loop thru all of the data bits:
;
checkEachBitInMorse   ; I think the problem might be here
		MOV 	R6, #0x0000 ;Only do this once per letter as we don't need to remove the zeroes again
		MOVT	R6, #0x8000	; Init R6 with the value for the bit, 15th, which we wish to test
		CLZ 	R11, R1				; count the Leading Zeros.  ;;here's the problem, we are not getting the right value for second char
		LSL     R1, R11 			; To get rid of the leading zeroes.
		MOV		R9,#32				; we have 32 bits in total, by substracting the number of leading zeors we can get how many bits we need to shitf later
		SUBS	R10,R9,R11			
		MOV		R9,R10						
		BL PostShift 	;To update the value of the link register
PostShift
		ANDS		R7, R1, R6			; R7 gets R1 AND R6, Zero bit gets set if it's zero			
		BLEQ		LED_OFF				; branch when it's zero
		BLNE		LED_ON          	; branch when it's not zero
		SUBS	    R9, #1				; Subtracting 1 from R9. To figure out how many shifts left
		LSL			R1, R1, #1	        ; get the most significant bit in R1
		;CMP         R9,#0
		BEQ			afterOneChar				
		B			PostShift    
; This is the end of the main program 




; Subroutines
;
;			convert ASCII character to Morse pattern
;			pass ASCII character in R0, output in R1
;			index into MorseLuT must be by steps of 2 bytes
CHAR2MORSE	STMFD		R13!,{R14}	; push Link Register (return address) on stack
				SUB				R2,R2,#0x41 		; convert the ASCII to an index (subtract 41). ASCII Starts 
				LSL				R2,R2, #1			; shift R0 left by 1, store in R2. Morse is in 16 bits and ASCII is in 8
				LDR				R1, =MorseLUT		; Pointing R1 to the address of MorseLUT.	
				ADD				R2,R2, R1			; Traversing through the alphabets.
		;		MOV				R1,#0x00000000
				LDRH			R1, [R2]  ;ontains the memory address for the morse code- store the contents of R0 into R1.
		    LDMFD		R13!,{R15}	; restore LR to R15 the Program Counter to return


; Turn the LED on, but deal with the stack in a simpler way
; NOTE: This method of returning from subroutine (BX  LR) does NOT work if subroutines are nested!!
;
LED_ON 	   	STMFD		R13!,{R3,R0,R4 ,R14};push 		{r3-r4}		; preserve R3 and R4 on the R13 stack
			MOV R3, #RED				; Move 8-bit immediate value into R0.load in the value to turn the RED led ON
			LDR R4, =GPIO_PORTF_DATA_R ; pointer to Port F data register
			STR R3, [R4]   	
			MOV			R0,#1	;Delay once.
			BL			DELAY
			LDMFD		R13!,{R3,R0,R4, R15};pop 		{r3-r4}

; Turn the LED off, but deal with the stack in the proper way
; the Link register gets pushed onto the stack so that subroutines can be nested
;
LED_OFF	   	STMFD		R13!,{R3,R0,R4, R14}	; push R3 and Link Register (return address) on stack
			MOV R3, #0			; load in the value to turn the RED led OFF
			LDR R4, =GPIO_PORTF_DATA_R ; pointer to Port F data register
			STR R3, [R4]
			MOV			R0,#1 ; Delay once.						
			BL			DELAY		
			LDMFD		R13!,{R3,R0,R4, R15}	; restore R3 and LR to R15 the Program Counter to return

starNextCycle
			BL			LED_OFF
			MOV 		R0, #4
			BL			DELAY
			B			ResetLUT
		ALIGN				; make sure things fall on word addresses
;	Delay 500ms * R0 times
;	Use the delay loop from Lab-1 but loop R0 times around
;
				
DELAY			STMFD		R13!,{R0,R1, R14}
				                   ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
ReDelay
				  LDR         R1,=SOMEDELAY ; R1 is the actual delay time
				  CMP  R0,#0
				  BNE ActualDelay
				  BEQ exitDelay
ActualDelay
                SUBS        R1,#1
                BNE         ActualDelay
				SUBS R0, R0, #1
                CMP  R0,#0                       
				BNE ReDelay
exitDelay		LDMFD		R13!,{R0,R1, R15}





afterOneChar		   
				BL LED_OFF
				MOV R0, #3
				BL DELAY
				B NextChar
;------------PortF_Init------------ 
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2
PortF_Init
    LDR R1, =SYSCTL_RCGCGPIO_R      ; 1) activate clock for Port F
    LDR R0, [R1]                 
    ORR R0, R0, #0x20               ; set bit 5 to turn on clock
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
    LDR R1, =GPIO_PORTF_LOCK_R      ; 2) unlock the lock register
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_CR_R        ; enable commit for Port F
    MOV R0, #0xFF                   ; 1 means allow access
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_AMSEL_R     ; 3) disable analog functionality
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_PCTL_R      ; 4) configure as GPIO
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1]                  
    LDR R1, =GPIO_PORTF_DIR_R       ; 5) set direction register
    MOV R0,#0x0E                    ; PF0 and PF7-4 input, PF3-1 output
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_AFSEL_R     ; 6) regular port function
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_PUR_R       ; pull-up resistors for PF4,PF0
    MOV R0, #0x11                   ; enable weak pull-up on PF0 and PF4
    STR R0, [R1]              
    LDR R1, =GPIO_PORTF_DEN_R       ; 7) enable Port F digital port
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1]                   
    BX  LR      

    ALIGN                           ; make sure the end of this section is aligned

;
; Data used in the program
; DCB is Define Constant Byte size
; DCW is Define Constant Word (16-bit) size
; EQU is EQUate or assign a value.  This takes no memory but instead of typing the same address in many places one can just use an EQU
;
		ALIGN				; make sure things fall on word addresses

; One way to provide a data to convert to Morse code is to use a string in memory.
; Simply read bytes of the string until the NULL or "0" is hit.  This makes it very easy to loop until done.
;
InputLUT	DCB		"SOS", 0	; strings must be stored, and read, as BYTES

		ALIGN				; make sure things fall on word addresses
MorseLUT 
		DCW 	0x17, 0x1D5, 0x75D, 0x75 	; A, B, C, D
		DCW 	0x1, 0x15D, 0x1DD, 0x55 	; E, F, G, H
		DCW 	0x5, 0x1777, 0x1D7, 0x175 	; I, J, K, L
		DCW 	0x77, 0x1D, 0x777, 0x5DD 	; M, N, O, P
		DCW 	0x1DD7, 0x5D, 0x15, 0x7 	; Q, R, S, T
		DCW 	0x57, 0x157, 0x177, 0x757 	; U, V, W, X
		DCW 	0x1D77, 0x775 			; Y, Z

    END                             ; end of file
