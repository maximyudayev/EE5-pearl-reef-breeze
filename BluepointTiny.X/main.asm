;*******************************************************************************
; FileName:        main.asm
; Processor:       PIC18F2550
; Compiler:        MPLAB® MPASMX
; Comment:         Main Assmebly code for CAN bus simulation board
;		   Identical logic is used for Node CAN functionality,
;		   but with different register names, values, filters, etc.
; Dependencies:    Header (p18f25k50.inc)
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Author		Date	    Version	Comment
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Maxim Yudayev		08/03/2020  0.1		-Verified in Debug mode:
;						  *correct CAN controller setup
;						  *correct TX buffer loading and
;						   request to send
;			09/03/2020  0.2		  *correct reaction to INT2,
;						   CAN controller interrupt flag
;						   clearing, interrupt reaction
;						   queuing and buffer reloading
;						   on successful transmission
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; TODO			Date        Finished	Comment
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; -Add CAN interrupt	09/03/2020  09/03/2020	-On SPI transaction completion,
;  queue to avoid				 if there are still uncleared
;  deadlocks					 flags in VCANINT, program takes
;						 care of all other interrupt
;						 sources separately
; -Verify proper	09/03/2020
;  real-life working
;  using logic analyzer
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Description
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; -In current implementation, to comply with 50ns CAN controller's IPT of /SS,
;  change of pin state is initiated a few instrucitons before writing data to
;  the SPI buffer - may be better in the future to replace with a precise timer,
;  but the current workaround is deemed sufficient
; -SPI operates in 0, 0 Mode
; -Optional /TRxRTS pins are configured
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Future Improvements
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; 1. SW configurable driven transmit priority
; 2. SW configurable receive masks/filters
; 3. SW configurable interrupt sources/events
; 4. Slave Select pin time-off for accurate 50ns IPT delay after IOC indicates
;    change of pin state
; 5. Self calibration to the unknown timing of the CAN bus (saving successful
;    calibration time quanta and configs to EEPROM) - selectable in SW for PIC18F
; 6. Higher level caller function to transmit real data
; 7. Indication in SW of filter match, rollover, priority, bus errors, etc.
;*******************************************************************************

;***********************************************************
; File Header
;***********************************************************
    
; PIC18F25K50 Configuration Bit Settings
; Assembly source line config statements
#include "p18f25k50.inc"

; CONFIG1L
  CONFIG  PLLSEL = PLL4X        ; PLL Selection (4x clock multiplier)
  CONFIG  CFGPLLEN = OFF	; PLL Enable Configuration bit (PLL Disabled (firmware controlled))
  CONFIG  CPUDIV = NOCLKDIV     ; CPU System Clock Postscaler (CPU uses system clock (no divide))
  CONFIG  LS48MHZ = SYS24X4     ; Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)

; CONFIG1H
  CONFIG  FOSC = ECHIO		; Oscillator Selection (EC oscillator, high power 16MHz to 48MHz)
  CONFIG  PCLKEN = ON           ; Primary Oscillator Shutdown (Primary oscillator enabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  nPWRTEN = OFF         ; Power-up Timer Enable (Power up timer disabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
  CONFIG  BORV = 190            ; Brown-out Reset Voltage (BOR set to 1.9V nominal)
  CONFIG  nLPBOR = OFF          ; Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

; CONFIG2H
  CONFIG  WDTEN = OFF           ; Watchdog Timer Enable bits (WDT enabled in hardware (SWDTEN ignored))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscaler (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = RC1          ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
  CONFIG  T3CMX = RC0           ; Timer3 Clock Input MUX bit (T3CKI function is on RC0)
  CONFIG  SDOMX = RC7		; SDO Output MUX bit (SDO function is on RB3)
  CONFIG  MCLRE = ON            ; Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
  CONFIG  LVP = ON              ; Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
  CONFIG  ICPRT = OFF           ; Dedicated In-Circuit Debug/Programming Port Enable (ICPORT disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)
  CONFIG  DEBUG = ON		; In-Circuit Debugging using PICKit3
  
; CONFIG5L
  CONFIG  CP0 = OFF             ; Block 0 Code Protect (Block 0 is not code-protected)
  CONFIG  CP1 = OFF             ; Block 1 Code Protect (Block 1 is not code-protected)
  CONFIG  CP2 = OFF             ; Block 2 Code Protect (Block 2 is not code-protected)
  CONFIG  CP3 = OFF             ; Block 3 Code Protect (Block 3 is not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protect (Boot block is not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protect (Data EEPROM is not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
  CONFIG  WRT1 = OFF            ; Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
  CONFIG  WRT2 = OFF            ; Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
  CONFIG  WRT3 = OFF            ; Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protect (Data EEPROM is not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)

CAN_RST		    equ 0x00
CAN_RD		    equ 0x01
CAN_RD_RX_BUF_0H    equ 0x02
CAN_RD_RX_BUF_0D    equ 0x03
CAN_RD_RX_BUF_1H    equ 0x04
CAN_RD_RX_BUF_1D    equ 0x05
CAN_WR		    equ 0x06
CAN_LD_TX_BUF_0H    equ 0x07
CAN_LD_TX_BUF_0D    equ 0x08
CAN_LD_TX_BUF_1H    equ 0x09
CAN_LD_TX_BUF_1D    equ 0x0A
CAN_LD_TX_BUF_2H    equ 0x0B
CAN_LD_TX_BUF_2D    equ 0x0C
CAN_RTS_0	    equ 0x0D
CAN_RTS_1	    equ 0x0E
CAN_RTS_2	    equ 0x0F
CAN_RD_ST	    equ 0x10
CAN_RX_ST	    equ 0x11
CAN_BM		    equ 0x12
VWREG		    equ	0x15
VCANINT		    equ	0x16
VEFLG		    equ 0x17		    
VISR		    equ 0x20	; Virtual Interrupt Service Register
				; bit 7 - carrying out SERCOM transaction, do not touch my data buffer yet
				; bit 6 -
				; bit 5 - one of CAN internal interrupts, check VCANINT and VEFLG for details
				; bit 4 - TMR0 interrupt, MCU waited for 8.3us to allow CAN controller start-up
				; bit 3 - IOCC2 interrupt, RX1BF changed pin state (SW reacts only on falling edge)
				; bit 2 - IOCC1 interrupt, RX0BF changed pin state (SW reacts only on falling edge)
				; bit 1 - INT2 interrupt, CAN controller has data
				; bit 0 - SPI interrupt, byte sent
CAN_CTRL_INIT	    equ 0x21	; CAN Controller Init Status Register
				; bit 7 - Ready
				; bit 6 - 
				; bit 5 - 
				; bit 4 - Transition to Normal Mode verified
				; bit 3 - Bit timing, interrupts and mode configured
				; bit 2 - Rx/Tx pins configured
				; bit 1 - RXB1CTRL configured
				; bit 0 - RXB0CTRL configured
COUNTER		    equ 0x22
SPI_CALLBACK_H	    equ 0x23	; Program memory address for SPI callback function
SPI_CALLBACK_L	    equ 0x24
CIRC_BUF_START	    equ 0x25	; Start of SPI buffer (static)
CIRC_BUF_END	    equ 0x26	; End of SPI buffer (dynamic, based on current action) 
BIT_MOD_CALLBACK_H  equ 0x27	; Program memory address for BIT MODIFY callback function
BIT_MOD_CALLBACK_L  equ 0x28
	    
;***********************************************************
; Reset Vector
;***********************************************************
 
    ORG	    0x0000		; 0x1000 in Production, 0x0000 in Debug/Simulation
    GOTO    save_instructions 
		
;***********************************************************
; Interrupt Vector
;***********************************************************

    ORG     0x0008		; 0x1008 in Production, 0x0008 in Debug/Simulation
    GOTO    inter_high	
    ORG     0x0018		; 0x1018 in Production, 0x0018 in Debug/Simulation
    GOTO    inter_low	
    
;***********************************************************
; Main
;***********************************************************
		     
    ORG     0x0020		; 0x1020 in Production, 0x0020 in Debug/Simulation

save_instructions
    movlw   0xC0
    movwf   CAN_RST
    movlw   0x03
    movwf   CAN_RD
    movlw   0x90
    movwf   CAN_RD_RX_BUF_0H
    movlw   0x92
    movwf   CAN_RD_RX_BUF_0D
    movlw   0x94
    movwf   CAN_RD_RX_BUF_1H
    movlw   0x96
    movwf   CAN_RD_RX_BUF_1D
    movlw   0x02
    movwf   CAN_WR
    movlw   0x40
    movwf   CAN_LD_TX_BUF_0H
    movlw   0x41
    movwf   CAN_LD_TX_BUF_0D
    movlw   0x42
    movwf   CAN_LD_TX_BUF_1H
    movlw   0x43
    movwf   CAN_LD_TX_BUF_1D
    movlw   0x44
    movwf   CAN_LD_TX_BUF_2H
    movlw   0x45
    movwf   CAN_LD_TX_BUF_2D
    movlw   0x81
    movwf   CAN_RTS_0
    movlw   0x82
    movwf   CAN_RTS_1
    movlw   0x83
    movwf   CAN_RTS_2
    movlw   0xA0
    movwf   CAN_RD_ST
    movlw   0xB0
    movwf   CAN_RX_ST
    movlw   0x05
    movwf   CAN_BM
    clrf    VISR		    ; Virtual Interrupt Service Register
    clrf    SPI_CALLBACK_L	    ; contains PC address of the routine to carry out on SPI transactions
    clrf    SPI_CALLBACK_H
    clrf    BIT_MOD_CALLBACK_H
    clrf    BIT_MOD_CALLBACK_L
    clrf    CAN_CTRL_INIT
    clrf    VWREG
    clrf    VCANINT
    clrf    VEFLG
    clrf    COUNTER		    ; counter is used to generate fake data
    movlw   0x30
    movwf   CIRC_BUF_START	    ; Circular Buffer starts at 0x20
    movwf   CIRC_BUF_END	    ; Circular Buffer end is specified by current action
 
config_hw
    movlw   0x80		    ; 3xPLL
    movwf   OSCTUNE		
    movlw   0x70		    ; HFINTOSC 16MHz, Sleep mode when issued SLEEP
    movwf   OSCCON		
    movlw   0x10		    ; PLLEN
    movwf   OSCCON2
    clrf    CM1CON0
    
    ; Disable USB
    bcf	    UCON,3		    ; disable USB module
    bsf	    UCFG,3		    ; disable internal USB transceiver
    
    clrf    LATA
    clrf    ANSELA
    movlw   0x00 		
    movwf   TRISA
    
    movlw   0x05		    ; RB0 = SDI , RC7 = SDO , RB1 = SCL, RB3 = SS, RB2 = CAN_INT
    movwf   TRISB		    ; Set PORTB as output
    movlw   0xEC		    ; Set SS, and CAN_INT
    movwf   LATB
    clrf    ANSELB
    
    clrf    LATC
    clrf    ANSELC
    movlw   0x00		    ; RC1 and RC6 inputs for RXnBF interrupt	       
    movwf   TRISC
    
    movlw   0x00
    movwf   SSP1STAT
    movlw   0x22
    movwf   SSP1CON1		    ; Fosc/64 as SPI clock source
    movlw   0x10
    movwf   SSP1CON3
    
    ; Initialize TMR0 interrupt; others after CAN controller oscillator start-up time (8.3us)
    movlw   0x48
    movwf   T0CON		    ; configure TMR0 as 120kHz (8.3us): 8bit, PS1:1, 156 offset, do not start yet
    movlw   0x9C
    movwf   TMR0L		    
    movlw   0x14
    movwf   INTCON2		    ; TMR0 interrupt high priority
    movlw   0x20
    movwf   INTCON		    ; enable TMR0 interrupt, do not enable global interrupt yet

;***********************************************************
; Initialize Fake Data
;***********************************************************
    
generate_fake_data
    lfsr    2, 0x0100

fake_data_loop			    ; loop to fill RAM with counter data 
    movff   COUNTER, POSTINC2
    incf    COUNTER
    movlw   0x00
    cpfseq  FSR2L		    ; when FSR2L overflows, 1 bank is full   
    bra	    fake_data_loop
	
;***********************************************************
; Main
;***********************************************************

; 128*63ns oscillator start-up delay on Reset = 8us,
; delay TMR0 will trigger after 8.3us with 8bit, PS1:1 and 156 offset to allow setup after that time
can_ctrlr_startup_delay
    bsf	    INTCON, 7		    ; enable global interrupt
    bsf	    T0CON, 7		    ; start TMR0
    bra	    main		    ; wait for TMR0 interrupt, enable all other interrupts afterwards
    
; 50ns minimum required delay for the transition of /SS line in both ways attach a timer if in practice undefined behavior is observed
; Buffer for CAN init occupies memory locations 0x030 - 0x045 (merely for convenience of keeping all register values together in can_ctrlr_init func)
can_ctrlr_init			    ; Configure CAN Controller for 500kbps, 40m bus and 69% NBT sampling
    bsf	    VISR, 7		    ; set SERCOM transaction bit to prevent buffer overwrite/interruption
    clrf    CAN_CTRL_INIT	    ; clear previous CAN INIT register state
    lfsr    1, 0x030		    ; load FSR1 with address of the first future instruction
    lfsr    2, 0x02A		    ; load FSR2 with address of the last transaction's buffer end pointer
    movlw   0x45
    movwf   POSTDEC2		    ; write in reverse order transaction buffer end pointers for use in callbacks
    movlw   0x42
    movwf   POSTDEC2		    ; these buffer end pointers are used to indicate the steps of the controller init procedure
    movlw   0x3C
    movwf   POSTDEC2
    movlw   0x35
    movwf   INDF2		    ; stop FSR2L at the location 0x027. It now spans cells 0x027-0x02A
    movlw   0x32		    
    movwf   CIRC_BUF_END	    ; update buffer start/end based on the first transaction (update start and end in SPI callback)
    movlw   0x31		    
    movwf   SPI_CALLBACK_H	    ; Provide SPI_CALLBACK at 0x3100
    movlw   0x00
    movwf   SPI_CALLBACK_L

; First SERCOM transaction
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x60		    
    movwf   POSTINC1		    ; RXB0CTRL address
    movlw   0xFF		    
    movwf   POSTINC1		    ; RXB0CTRL - configures all masks/filters off for RXB0 to receive all messages and truns on rollover to RXB1

; New SERCOM transaction
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x70
    movwf   POSTINC1		    ; RXB1CTRL address
    movlw   0xFF
    movwf   POSTINC1		    ; RXB1CTRL - configures all masks/filters off for RXB1

; New SERCOM transaction    
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x28
    movwf   POSTINC1		    ; CNF3 address
    movlw   0x03		    
    movwf   POSTINC1		    ; CNF3 (0x28): PHSEG2 is 4Tq, wake-up filter disabled
    movlw   0xDE		    
    movwf   POSTINC1		    ; CNF2 (0x29): Bus sampled 3 times at sample point, PHSEG1 is 4Tq, PRSEG is 7Tq
    movlw   0xC0		    
    movwf   POSTINC1		    ; CNF1 (0x2A): Sync Jump Width of 4Tq, Baud Rate prescaler of 0
    movlw   0x3F		    
    movwf   POSTINC1		    ; CANINTE (0x2B): Enable interrupts on buffer transmission completion, receiver buffer full and bus errors from EFLG register
    movlw   0x00		    
    movwf   POSTINC1		    ; CANINTF (0x2C): Clear all interrupt flags

; New SERCOM transaction
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x0C
    movwf   POSTINC1		    ; BFPCTRL address
    movlw   0x00			    
    movwf   POSTINC1		    ; BFPCTRL (0x0C) - disables /RXnBF not to trigger interrupt on buffer full
    movlw   0x07			    
    movwf   POSTINC1		    ; TXRTSCTRL (0x0D)- configures /TXnRTS to trigger transmission on falling edge
    movlw   0x00			    
    movwf   POSTINC1		    ; CANSTAT (0x00) - Read-only, written for easy uninterrupted sequential clocking of data
    movwf   POSTINC1		    ; CANCTRL (0x0F) - Sets Normal Mode, desables One Shot Mode and CLKOUT pin
    
; New SERCOM transaction    
    movff   CAN_RD, POSTINC1	    ; READ instruction
    movlw   0x0E
    movwf   POSTINC1		    ; CANSTAT address
    movwf   POSTINC1		    ; Check CANSTAT to verify entering of Normal Mode (dummy data on MOSI is okay)
    
; Initiate 1st SERCOM transaction
    bcf	    LATB, 3		    ; Pull /SS line low
    lfsr    1, 0x030		    ; set FSR1 back at the beginning of the buffer
    movff   POSTINC1, SSP1BUF	    ; Start transmission and enter "main"
				    
main
    movf    VISR, 0		    ; load VISR to WREG
    andlw   0x7F		    ; binary AND with 0b01111111
    bz	    main		    ; if zero (no software flags set), remain in loop, else enter "action"

action
    btfsc   VISR, 0		    ; is SPI triggered?
    bra	    spi_action
    btfsc   VISR, 7		    ; if serial transaction of data buffer is in progress, wait for completion
    bra	    main		    
    btfsc   VISR, 1		    ; is INT2 triggered?
    bra	    can_action		    
    btfsc   VISR, 4		    ; is TMR0 triggered?
    bra	    can_startup_action	    ; if none of the above, then go to ioc_action
    
ioc_action			    ; Triggered when RXnBF pin state changes (SW ISR marks only transition on falling edge)    
    lfsr    1, 0x030		    ; else, based on the event source, load proper instruction into the buffer
    lfsr    2, 0x040		    ; Load FSR2 with 0x040 for saving returned data
    clrf    COUNTER		    ; Counter here counts bytes received
    btfsc   VISR, 2		    ; if RC1 is source
    bra	    can_read_rxbuf0	    ; else RC6 is source

can_read_rxbuf1
    bcf	    VISR, 3	
    movff   CAN_RD_RX_BUF_1H, INDF1
    bra	    can_spibufld_rxbuf
    
can_read_rxbuf0
    bcf	    VISR, 2
    movff   CAN_RD_RX_BUF_0H, INDF1
    
can_spibufld_rxbuf		    ; Puts 13 bytes of dummy data into circular buffer
    movlw   0x2E		    ; by setting length of buffer to 13 bytes + 1 (for easy XOR)
    movwf   CIRC_BUF_END
    movlw   0x30		    ; Load SPI_CALLBACK with program memory address of "can_read_rxbuf_cb"
    movwf   SPI_CALLBACK_H
    movlw   0x00
    movwf   SPI_CALLBACK_L
    bsf	    VISR, 7		    ; Set VISR<7> to prevent other software interrupts from overwriting SPI SW buffer
    bcf	    LATB, 3		    ; Pull Slave Select line low (verify hold timing on oscilloscope, add timer interrupt if otherwise not working properly)
    movff   POSTINC1, SSP1BUF	    ; Initiate SPI transmission by loading the first SW buffer byte
    bra	    main		    ; Return to "main" and wait for new interrupts
    
spi_action			    ; triggered when SPI transmission of 1 byte is finished, invokes callback
    bcf	    VISR, 0		    ; remove SPI software flag
    clrf    PCLATU		    ; update the PC with location of callback function
    movff   SPI_CALLBACK_H, PCLATH
    movf    SPI_CALLBACK_L, 0
    movwf   PCL			    ; after this instruciton, program should jump to appropriate callback

can_startup_action		    ; triggered when TMR0 count 8.3us indicating end of CAN oscillator startup time
    clrf    T0CON		    ; turn off TMR0
    bcf	    VISR, 4		    ; remove software flag
    movlw   0x40		    ; configure interrupts for normal operation
    movwf   INTCON		    ; enable external interrupt and interrupt on pin change, enable interrupts from peripherals
    movlw   0xE0
    movwf   INTCON2		    ; enable external INT2 on falling edge
    movlw   0x90
    movwf   INTCON3		    ; enable external INT2
;    movlw   0x42		    
;    movwf   IOCC		    ; enable on pin change on pin RC1 and RC6
    movlw   0x08
    movwf   PIE1		    ; interupt enable SPI
    clrf    PIR1		    ; clear interrupt flags
    bsf	    INTCON, 7		    ; enable global interrupt    
    bra	    can_ctrlr_init
    
can_action			    ; triggered when INT2 is pulled down by CAN controller
    bsf	    VISR, 7		    ; indicate start of SERCOM transaction
    bcf	    VISR, 1		    ; remove software flag for external INT2
    lfsr    1, 0x038		    ; set FSR1 at the start of interrupt source checking bytes
    bcf	    LATB, 3		    ; pull /SS line low, notify Slave
    movlw   0x34
    movwf   SPI_CALLBACK_H	    ; provide SPI callback
    movlw   0x00
    movwf   SPI_CALLBACK_L
    movlw   0x3A
    movwf   CIRC_BUF_END	    ; set end of buffer to be the end of SPI transmission when CANINTF is received 
    movff   POSTINC1, SSP1BUF	    ; initiate SPI transmission
    goto    main
    
;***********************************************************
; SPI Callbacks
;***********************************************************
    
    ORG	    0x3000			    

can_read_rxbuf_cb		    ; process data returned from the CAN RX Buffer n (0 | 1)
    movlw   0x04
    xorwf   COUNTER, 0		    ; check if current byte is RXBnDLC
    bnz	    can_read_rxbuf_cb_nondlc
    movlw   0x36
    addwf   SSP1BUF, 0;		    
    movwf   CIRC_BUF_END	    ; adjust buffer length based on RXBnDLC value
    
can_read_rxbuf_cb_nondlc
    movff   SSP1BUF, POSTINC2
    incf    COUNTER
    movf    FSR1L, 0
    xorwf   CIRC_BUF_END	    ; check if FSR1L reached X + 1 bytes
    bnz	    can_read_rxbuf_cb_cont		    
				    ; SERCOM transaction finished - clean up
    bsf	    LATB, 3		    ; pull high the Slave Select line
    movff   CIRC_BUF_START, CIRC_BUF_END    ; set CIRC_BUF_END equal to CIRC_BUF_START
    bcf	    VISR, 7		    ; clear VISR<7>
    goto    main		    ; return to main
    
can_read_rxbuf_cb_cont
    movff   POSTINC1, SSP1BUF
    goto    main
    
    
    ORG	    0x3100

can_init_cb
    incf    CIRC_BUF_END, 0
    xorwf   FSR1L, 0
    bz	    can_init_msg_fin	    ; check if FSR1 reached end of buffer for current transaction
    movff   POSTINC1, SSP1BUF	    ; if not, send next byte
    goto    main
    
can_init_msg_fin
    bsf	    LATB, 3		    ; pull /SS line back high, release Slave
    btfsc   CAN_CTRL_INIT, 3	    ; if bit-3 (indication of transition to Normal Mode) is set and new transaction finished
    bra	    can_init_modechk	    ; it indicates that SSP1BUF contains CANSTAT returned value
    movff   POSTINC2, CIRC_BUF_END  ; update the end of the buffer with the next memory address
    bcf	    LATB, 3		    ; pull /SS line back low, notify Slave
    rlncf   CAN_CTRL_INIT, 1
    incf    CAN_CTRL_INIT, 1	    ; rotate left and increment the INIT register of CAN
    movff   POSTINC1, SSP1BUF	    ; initiate the next transaction
    goto    main

can_init_modechk_fail
    btg	    PORTC, 2
    goto    can_ctrlr_init	    ; if not, try reinitializing all over again
    
can_init_modechk
    movf    SSP1BUF, 0		    
    movwf   LATA
    andlw   0xE0		    ; mask CANSTAT with 0b11100000 to get only OPMOD[2:0] bits   
    bnz	    can_init_modechk_fail   ; verify that CANSTAT matches expected value of Normal mode
    bsf	    CAN_CTRL_INIT, 4	    ; if configuration succeeded, indicate in CAN INIT register
    bsf	    CAN_CTRL_INIT, 7	    ; then proceed to writing data to CAN controller in 'can_write_txbuf' func

; setup header data in data memory for each transmit message
    lfsr    2, 0x100		    ; set FSR2 at the beginning of the dummy data bank 1
    lfsr    1, 0x030		    ; fill up data memory addresses 0x30-0x37 with transmit message header data
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x30
    movwf   POSTINC1		    ; Address of TXB0CTRL
    clrf    POSTINC1		    ; TXBnCTRL: remove TXREQ flag bit, TXP[1:0] = 00 -> lowest priority
    movlw   0xAA		    ; load buffer with ID and DLC
    movwf   POSTINC1		    ; SID[10:0] = 0b10101010101
    movlw   0xA0
    movwf   POSTINC1		    ; EXIDE = 0 -> Standard Identifier used
    clrf    POSTINC1		    ; EXIDL unimplemented
    clrf    POSTINC1		    ; EXIDH unimplemented
    movlw   0x08
    movwf   POSTINC1		    ; DLC = 8 bytes
; fill up 0x38-0x3B with interrupt source checking bytes
    movff   CAN_RD, POSTINC1	    ; READ instruction
    movlw   0x2C
    movwf   POSTINC1		    ; address of CANINTF
    clrf    POSTINC1		    ; do not care data for CANINTF
    clrf    POSTINC1		    ; do not care data for EFGL
; fill up 0x3C-0x3F with interrupt source checking bytes    
    movff   CAN_BM, POSTINC1	    ; BIT MODIFY instruction
    movlw   0x2C
    movwf   POSTINC1		    ; address of CANINTF
    movlw   0x04		    ; mask byte: initially allows only TX0IF to be changed
    movwf   POSTINC1		    ; it is allowed to update it during program operation
    movlw   0x00		    ; data byte: clears all masked bits
    movwf   POSTINC1		    ; it is allowed to update it during program operation
; continue immediately to "can_write_txbuf" to start transmission of messages
    
can_write_txbuf
    lfsr    1, 0x030
    bcf	    LATB, 3		    ; notify Slave of an incoming message
    clrf    COUNTER		    ; clear counter (to count off 8 data bytes of FSR2)
    movlw   0x37
    movwf   CIRC_BUF_END	    ; set buffer length to 8 (1 instruction, 1 memory address, 1 CTRL register value, 4 ID bytes and 1 DLC)
    movlw   0x32
    movwf   SPI_CALLBACK_H	    ; provide SPI callback
    movlw   0x00
    movwf   SPI_CALLBACK_L
    movff   POSTINC1, SSP1BUF	    ; load first byte into SPI buffer, initiate SERCOM transaction
    goto    main
    
    
    ORG	    0x3200
    
can_write_txbuf_cb
    incf    CIRC_BUF_END, 0
    xorwf   FSR1L, 0
    bz	    can_write_txbuf_data    ; check if FSR1 reached the end of buffer for current transaction
    movff   POSTINC1, SSP1BUF	    ; if not, send next byte
    goto    main

can_write_txbuf_data
    movlw   0x33		    
    movwf   SPI_CALLBACK_H	    ; update SPI callback with ..._data_cb
    movlw   0x00
    movwf   SPI_CALLBACK_L
    movff   POSTINC2, SSP1BUF	    ; send the first data byte to TX buffer
    incf    COUNTER, 1		    ; increment number of sent data bytes
    goto    main		    ; return to main
    
    
    ORG	    0x3300
    
can_write_txbuf_data_cb
    movlw   0x08
    xorwf   COUNTER, 0		    ; check if all 8 data bytes were written to TX buffer
    bz	    can_write_txbuf_msg_fin
    incf    COUNTER, 1		    ; increment the number of already sent data bytes
    movlw   0x02
    xorwf   FSR2H, 0		    ; check if FSR2 looped to the end of bank 1
    btfsc   STATUS, 2		    ; Z bit of STATUS register is 1 in this case
    lfsr    2, 0x100		    ; set FSR2 back to the beginning of bank 1 (circular buffer style)
    movff   POSTINC2, SSP1BUF	    ; send next data byte
    goto    main		    ; return to main
    
can_write_txbuf_msg_fin
    bsf	    LATB, 3		    ; release Slave to indicate end of SPI command
    bcf	    LATB, 5		    ; pull low the TX Buffer 0 RTS pin to start CAN transmission
    movf    VCANINT, 0
    bnz	    can_isr_src_list	    ; if there is a queue of other interrupts, treat them
    bcf	    VISR, 7		    ; else, indicate end of SERCOM transaction, wait for Transmit Completed message from CAN
    goto    main
    
    
    ORG	    0x3400

can_isr_src_cb
    incf    CIRC_BUF_END, 0
    xorwf   FSR1L, 0
    bz	    can_isr_src_msg_fin
    movff   POSTINC1, SSP1BUF
    goto    main
    
can_isr_src_msg_fin		    
    movlw   0x3B
    xorwf   FSR1L, 0		    ; transition between CANINTF and EFLG?
    bnz	    can_isr_src_tst	    ; if not, branch
    movff   SSP1BUF, VCANINT	    ; else, save value of CANINTF
    btfsc   VCANINT, 5		    ; and check if EFLG also triggered interrupt
    bra	    can_isr_src_chkeflg	    ; if so, send another byte and record EFLG value to VEFLG
				    ; else, stop transaction and continue to BIT MODIFY logic

can_isr_src_tst
    bsf	    LATB, 3		    ; release Slave to indicate end of SPI instruction
can_isr_src_list		    ; TODO: implement callbacks for all interrupt sources
    movf    VCANINT, 0
    andlw   0x20		    ; was EFLG one of interrupt sources?
    bnz	    can_isr_src_eflg	    ; if EFLG is one of sources, branch
    btfsc   VCANINT, 2		    ; else test other sources in VCANINT
    bra	    can_isr_tx0_success	     
    btfsc   VCANINT, 3
    bra	    can_isr_tx1_success	    
    btfsc   VCANINT, 4
    bra	    can_isr_tx2_success
    btfsc   VCANINT, 0
    bra	    can_isr_rx0_rcvd	    
    btfsc   VCANINT, 1
    bra	    can_isr_rx1_rcvd
    ; should always trigger one of the above conditions. Each function affects corresponding mask and data bit

can_isr_src_chkeflg
    movlw   0x3B
    movwf   CIRC_BUF_END	    ; extend buffer length by 1 more byte
    movff   POSTINC1, SSP1BUF	    ; send dummy data in exchange for EFLG value
    goto    main
    
can_isr_tx0_success		    
    bsf	    LATB, 5		    ; pull high TX0RTS line
    movlw   0x04
    movwf   0x3E		    ; apply mask for TX0IF
    movlw   0x00
    movwf   0x3F		    ; clear masked flag
    bcf	    VCANINT, 2		    ; clear virtual CANINT.TX0IF flag
    bra	    can_isr_tx_set_bm_cb
    
can_isr_tx1_success
    bsf	    LATB, 6		    ; pull high TX1RTS line
    movlw   0x08
    movwf   0x3E		    ; apply mask for TX1IF
    movlw   0x00
    movwf   0x3F		    ; clear masked flag
    bcf	    VCANINT, 3		    ; clear virtual CANINT.TX1IF flag
    bra	    can_isr_tx_set_bm_cb
    
can_isr_tx2_success
    bsf	    LATB, 7		    ; pull high TX2RTS line
    movlw   0x10
    movwf   0x3E		    ; apply mask for TX2IF
    movlw   0x00
    movwf   0x3F		    ; clear masked flag
    bcf	    VCANINT, 4		    ; clear virtual CANINT.TX2IF flag
    
can_isr_tx_set_bm_cb
    movlw   0x36
    movwf   BIT_MOD_CALLBACK_H
    movlw   0x00
    movwf   BIT_MOD_CALLBACK_L
    bra	    can_isr_bit_mod
    
can_isr_rx0_rcvd		    ; in receive registers, check for overflow indicated by EFLG
    ; complete logic
    
can_isr_rx1_rcvd
    ; complete logic
    
can_isr_src_eflg		    ; check the source of interrupt, update mask and data. bit modify func will do everything else
    movff   SSP1BUF, VEFLG	    ; save EFLG value to VEFLG
    ; complete logic
    
can_isr_bit_mod			    ; one of functions above updated mask and data values, use them to modify the registers
    lfsr    1, 0x3C		    ; carried out separately on each source to avoid double interrupt counting as single if previous not yet resolved from software queue
    movlw   0x3F
    movwf   CIRC_BUF_END	    ; update buffer end with end of BIT MODIFY transaction
    bcf	    LATB, 3		    ; notify Slave of incoming SPI message
    movlw   0x35
    movwf   SPI_CALLBACK_H	    ; update SPI callback
    movlw   0x00
    movwf   SPI_CALLBACK_L
    bsf	    VISR, 7		    ; set SERCOM transaction flag
    movff   POSTINC1, SSP1BUF	    ; send first SPI data byte
    goto    main
    
    
    ORG	    0x3500
    
can_isr_bit_mod_cb
    incf    CIRC_BUF_END, 0
    xorwf   FSR1L, 0
    bz	    can_isr_bit_mod_fin	    ; continue sending next byte until 4 total bytes are sent 0x3C-0x3F
    movff   POSTINC1, SSP1BUF
    goto    main
    
can_isr_bit_mod_fin
    bsf	    LATB, 3		    ; notify Slave of the end of SPI command
    clrf    PCLATU		    ; update the PC with location of callback function
    movff   BIT_MOD_CALLBACK_H, PCLATH
    movf    BIT_MOD_CALLBACK_L, 0
    movwf   PCL			    ; after this instruciton, program should jump to appropriate callback
    
    
    ORG	    0x3600
    
can_tx_bit_mod_cb		    ; BIT MODIFY instruction cleared the TXnIF, now load that buffer
    rlncf   0x3E, 0		    ; load to WREG and shift left the bit mask (contains index of TX buffer whose interrupt was just cleared)
    andlw   0xF0		    ; keep the first neeble only
    addlw   0x30		    ; add with 0b00110000 offset to get TXBnCTRL register address
    movwf   0x31		    ; load WREG value to the Transmit Buffer X register to write to this, now ready, transmit buffer
    bra	    can_write_txbuf	    ; invoke write function to this ready TX buffer
    
;***********************************************************
; Interrupts
;***********************************************************
	     
inter_high
    btfsc   PIR1, 3
    bra	    spi_interrupt
    btfsc   INTCON3, 1
    bra	    can_interrupt
    btfsc   INTCON, 0
    bra	    ioc_interrupt
    btfsc   INTCON, 2
    bra	    tmr0_interrupt
    retfie
    
spi_interrupt
    bsf	    VISR, 0		    ; set corresponding software flag
    bcf	    PIR1, 3		    ; remove hardware flag
    retfie
    
can_interrupt
    bsf	    VISR, 1		    ; set corresponding software flag
    bcf	    INTCON3, 1		    ; remove hardware flag
    retfie

ioc_interrupt
    btfss   PORTC, 1
    bsf	    VISR, 2		    ; set corresponding software flag
    btfss   PORTC, 6		    
    bsf	    VISR, 3		    ; set corresponding software flag
    bcf	    INTCON, 0		    ; remove hardware flag
    retfie
    
tmr0_interrupt
    bsf	    VISR, 4		    ; set corresponding software flag
    bcf	    INTCON, 5		    ; disable TMR0 interrupt
    bcf	    INTCON, 2		    ; remove hardware flag
    retfie
    
inter_low
    nop
    retfie
    
    END