;*******************************************************************************
; FileName:        main.asm
; Processor:       PIC18F2550
; Compiler:        MPLAB® MPASMX
; Comment:         Main Assmebly code for CAN bus simulation board
;		   Identical logic is used for Node CAN functionality,
;		   but with different register names, values, filters, etc.
; Dependencies:    Header (p18f25k50.inc)
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Author		Date	    Version	    Comment
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Maxim Yudayev		08/03/2020  0.1		    -Verified in Debug mode
;						     correct CAN controller setup
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; TODO			Date        Finished	    Comment
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; -Add #define driven	08/03/2020  
;  masks/filters and
;  transmit priorities
;  for use by SolarTeam
;  as API
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Description
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;*******************************************************************************

;***********************************************************
; File Header
;***********************************************************
    
    list p=18F25k50, r=hex, n=0
    #include <p18f25k50.inc>
    
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
COUNTER		    equ 0x20
CAN_CTRL_INIT	    equ 0x21	; CAN Controller Init Status Register
				; bit 7 - Ready
				; bit 6 - 
				; bit 5 - 
				; bit 4 - Transition to Normal Mode verified
				; bit 3 - Bit timing, interrupts and mode configured
				; bit 2 - Rx/Tx pins configured
				; bit 1 - RXB1CTRL configured
				; bit 0 - RXB0CTRL configured
VISR		    equ 0x22	; Virtual Interrupt Service Register
				; bit 7 - carrying out SERCOM transaction, do not touch my data buffer yet
				; bit 6 -
				; bit 5 -
				; bit 4 - TMR0 interrupt, MCU waited for 8.3us to allow CAN controller start-up
				; bit 3 - IOCC2 interrupt, RX1BF changed pin state (SW reacts only on falling edge)
				; bit 2 - IOCC1 interrupt, RX0BF changed pin state (SW reacts only on falling edge)
				; bit 1 - INT2 interrupt, CAN controller has data
				; bit 0 - SPI interrupt, byte sent
SPI_CALLBACK_L	    equ 0x23	; Program memory address for SPI callback function
SPI_CALLBACK_H	    equ 0x24
CIRC_BUF_START	    equ 0x25	; Start of SPI buffer (static)
CIRC_BUF_END	    equ 0x26	; End of SPI buffer (dynamic, based on current action) 
	    
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
    clrf    CAN_CTRL_INIT
    clrf    COUNTER		    ; counter is used to generate fake data
    movlw   0x30
    movwf   CIRC_BUF_START	    ; Circular Buffer starts at 0x20
    movwf   CIRC_BUF_END	    ; Circular Buffer end is specified by current action
 
config_hw
    movlw   0x80		    ; 3xPLL
    movwf   OSCTUNE		
    movlw   0x72		    ; HFINTOSC 16MHz, Sleep mode when issued SLEEP
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
    movlw   0x42		    ; RC1 and RC6 inputs for RXnBF interrupt	       
    movwf   TRISC
    
    movlw   0x00
    movwf   SSP1STAT
    movlw   0x21
    movwf   SSP1CON1		    ; Fosc/16 as SPI clock source
    movlw   0x10
    movwf   SSP1CON3
    
    ; Initialize TMR0 interrupt; others after CAN controller oscillator start-up time (8.3us)
    movlw   0x48
    movwf   T0CON		    ; configure TMR0 as 120kHz (8.3us): 8bit, PS1:1, 156 offset, do not start yet
    movlw   0x9C
    movwf   TMR0L		    
    movlw   0x04
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
    movlw   0x30		    
    movwf   SPI_CALLBACK_H	    ; Provide SPI_CALLBACK
    movlw   0x2C
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
    movlw   0x03		    ; CNF3 (0x28): PHSEG2 is 4Tq, wake-up filter disabled
    movwf   POSTINC1
    movlw   0xDE		    ; CNF2 (0x29): Bus sampled 3 times at sample point, PHSEG1 is 4Tq, PRSEG is 7Tq
    movwf   POSTINC1
    movlw   0xC0		    ; CNF1 (0x2A): Sync Jump Width of 4Tq, Baud Rate prescaler of 0
    movwf   POSTINC1
    movlw   0x3F		    ; CANINTE (0x2B): Enable interrupts on buffer transmission completion, receiver buffer full and bus errors from EFLG register
    movwf   POSTINC1
    movlw   0x00		    ; CANINTF (0x2C): Clear all interrupt flags
    movwf   POSTINC1

; New SERCOM transaction
    movff   CAN_WR, POSTINC1	    ; WRITE instruction
    movlw   0x0C
    movwf   POSTINC1		    ; BFPCTRL address
    movlw   0x0F			    
    movwf   POSTINC1		    ; BFPCTRL (0x0C) - configures /RXnBF to trigger interrupt on buffer full
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
    btfsc   VISR, 1		    ; is INT2 triggered?
    bra	    can_action		    
    btfsc   VISR, 7		    ; if serial transaction of data buffer is in progress, wait for completion
    bra	    main		    
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
    clrf    SPI_CALLBACK_L
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
    movlw   0x58		    ; configure interrupts for normal operation
    movwf   INTCON		    ; enable external interrupt and interrupt on pin change, enable interrupts from peripherals
    movlw   0xE8
    movwf   INTCON2		    ; enable external INT2 on falling edge
    movlw   0x90
    movwf   INTCON3		    ; enable external INT2
    movlw   0x42		    
    movwf   IOCC		    ; enable on pin change on pin RC1 and RC6
    movlw   0x08
    movwf   PIE1		    ; interupt enable SPI
    clrf    PIR1		    ; clear interrupt flags
    bsf	    INTCON, 7		    ; enable global interrupt    
    bra	    can_ctrlr_init
    
can_action			    ; triggered when INT2 is pulled down by CAN controller
    ; check the source of interrupt
    ; process the trigger and check current MCU state
    ; add corresponding logic for multiple cases (i.e failed transaction, succes, etc)
    
;***********************************************************
; SPI Callbacks
;***********************************************************
    
    ORG	0x3000			    

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
    
    
    ORG 0x302C
    
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

can_init_modechk
    movf    SSP1BUF, 0		    
    xorlw   0xE0		    ; mask CANSTAT with 0b11100000 to get only OPMOD[2:0] bits   
    btfss   STATUS, 2		    ; verify that CANSTAT matches expected value of Normal mode
    goto    can_ctrlr_init	    ; if not, try reinitializing all over again
    bsf	    CAN_CTRL_INIT, 4	    ; if configuration succeeded, indicate in CAN INIT register
    bsf	    CAN_CTRL_INIT, 7	    ; then proceed to writing data to CAN controller in 'can_write_txbuf' func
				    ; cleanup
    movff   CIRC_BUF_START, CIRC_BUF_END    ; set buffer length to 0

can_write_txbuf
    ; write logic for writing data to CAN controller
    
;***********************************************************
; Interrupts
;***********************************************************
	     
inter_high
    btfsc   PIR1, 3
    call    spi_interrupt
    btfsc   INTCON3, 1
    call    can_interrupt
    btfsc   INTCON, 0
    call    ioc_interrupt
    btfsc   INTCON, 2
    call    tmr0_interrupt
    retfie
    
spi_interrupt
    bsf	    VISR, 0		    ; set corresponding software flag
    bcf	    PIR1, 3		    ; remove hardware flag
    return
    
can_interrupt
    bsf	    VISR, 1		    ; set corresponding software flag
    bcf	    INTCON3, 1		    ; remove hardware flag
    return

ioc_interrupt
    btfss   PORTC, 1
    bsf	    VISR, 2		    ; set corresponding software flag
    btfss   PORTC, 6		    
    bsf	    VISR, 3		    ; set corresponding software flag
    bcf	    INTCON, 0		    ; remove hardware flag
    return
    
tmr0_interrupt
    bsf	    VISR, 4		    ; set corresponding software flag
    bcf	    INTCON, 5		    ; disable TMR0 interrupt
    bcf	    INTCON, 2		    ; remove hardware flag
    return
    
inter_low
    nop
    retfie
    
    END