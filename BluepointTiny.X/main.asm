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
counter		    equ 0x15
VISR		    equ 0x16	; Virtual Interrupt Service Register
				; bit 7 - carrying out SERCOM transaction, do not touch my data buffer yet
				; bit 6 -
				; bit 5 -
				; bit 4 -
				; bit 3 - IOCC2 interrupt, RX1BF changed pin state (SW reacts only on falling edge)
				; bit 2 - IOCC1 interrupt, RX0BF changed pin state (SW reacts only on falling edge)
				; bit 1 - INT2 interrupt, CAN controller has data
				; bit 0 - SPI interrupt, byte sent
SPI_CALLBACK_L	    equ 0x17	; Program memory address for SPI callback function
SPI_CALLBACK_H	    equ 0x18
CIRC_BUF_START	    equ 0x20	; Start of SPI buffer (static)
CIRC_BUF_END	    equ 0x21	; End of SPI buffer (dynamic, based on current action) 
		
;***********************************************************
; Reset Vector
;***********************************************************
 
    ORG	    0x1000
    GOTO    save_instructions 
		
;***********************************************************
; Interrupt Vector
;***********************************************************

    ORG     0x1008	
    GOTO    inter_high	
    ORG     0x1018	
    GOTO    inter_low	
    
;***********************************************************
; Main
;***********************************************************
		     
    ORG     0x1020

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
    movlw   0x30
    movwf   CIRC_BUF_START	    ; Circular Buffer starts at 0x20
    movwf   CIRC_BUF_END	    ; Circular Buffer end is specified by current action
    clrf    counter		    ; counter is used to generate fake data
 
config_hw
    movlw   0x80		    ; load value 0x80 in work register
    movwf   OSCTUNE		
    movlw   0x70		    ; load value 0x70 in work register
    movwf   OSCCON		
    movlw   0x00		    ; load value 0x10 to work register
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
    
    ; Initialize interrupts
    movlw   0x80
    movwf   RCON		    ; Set RCON<7> (IPEN) to enable all peripheral interrupts
    
    movlw   0x18		    ; enable external interrupt and interrupt on pin change 
    movwf   INTCON
    movlw   0xE8
    movwf   INTCON2		    ; enable external INT2 on falling edge
    movlw   0x90
    movwf   INTCON3		    ; Enable external INT2
    
    movlw   0x42		    ; Enable on pin change on pin RC1 and RC6
    movwf   IOCC
    
    movlw   0x08
    movwf   PIE1		    ; Interupt enable SPI
    
    clrf    PIR1		    ; Clear interrupt flags
  
    bsf	    INTCON, 7		    ; Enable global interrupt

;***********************************************************
; Initialize Fake Data
;***********************************************************
    
generate_fake_data
    lfsr    2, 0x0100

fake_data_loop			    ; loop to fill RAM with counter data 
    movff   counter, POSTINC2
    incf    counter
    movlw   0x00
    cpfseq  FSR2L		    ; when FSR2L overflows, 1 bank is full   
    bra	    fake_data_loop
	
;***********************************************************
; Main
;***********************************************************

; Write here logic for setting up the CAN controller, then enter "main" when done
can_ctrlr_init
    bsf	    VISR, 7		    ; set SERCOM transaction bit to prevent interruption
    lfsr    1, 0x030
    movff   
     
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
    bra	    main		    ; if none of the above, then go to ioc_action
    
ioc_action			    ; triggered when RXnBF pin state changes (SW ISR marks only transition on falling edge)    
    lfsr    1, 0x030		    ; else, based on the event source, load proper instruction into the buffer
    lfsr    2, 0x040		    ; load FSR2 with 0x040 for saving returned data
    clrf    counter		    ; counter here counts bytes received
    btfsc   VISR, 2		    ; if RC1 is source
    bra	    can_read_rxbuf0	    ; else RC6 is source

can_read_rxbuf1
    bcf	    VISR, 3	
    movff   CAN_RD_RX_BUF_1H, INDF1
    bra	    can_spibufld_rxbuf
    
can_read_rxbuf0
    bcf	    VISR, 2
    movff   CAN_RD_RX_BUF_0H, INDF1
    
can_spibufld_rxbuf		    ; puts 13 bytes of dummy data into circular buffer
    movlw   0x2E		    ; by setting length of buffer to 13 bytes + 1 (for easy XOR)
    movwf   CIRC_BUF_END
    movlw   0x30		    ; load SPI_CALLBACK with program memory address of "can_read_rxbuf_cb"
    movwf   SPI_CALLBACK_H
    clrf    SPI_CALLBACK_L
    bsf	    VISR, 7		    ; set VISR<7> to prevent other software interrupts from overwriting SPI SW buffer
    bcf	    LATB, 3		    ; pull Slave Select line low (verify hold timing on oscilloscope, add timer interrupt if otherwise not working properly)
    movff   POSTINC1, SSP1BUF	    ; initiate SPI transmission by loading the first SW buffer byte
    bra	    main		    ; return to "main" and wait for new interrupts
    
spi_action			    ; triggered when SPI transmission of 1 byte is finished, invokes callback
    bcf	    VISR, 0		    ; clear SPI software flag
    clrf    PCLATU		    ; update the PC with location of callback function
    movff   SPI_CALLBACK_H, PCLATH
    movf    SPI_CALLBACK_L, 0
    movwf   PCL			    ; after this instruciton, program should jump to appropriate callback
    
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
    xorwf   counter, 0		    ; check if current byte is RXBnDLC
    bnz	    can_read_rxbuf_cb_nondlc
    movlw   0x36
    addwf   SSP1BUF, 0;		    
    movwf   CIRC_BUF_END	    ; adjust buffer length based on RXBnDLC value
    
can_read_rxbuf_cb_nondlc
    movff   SSP1BUF, POSTINC2
    incf    counter
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
    
;    ORG 0x3100
;    
;can_read_rxbuf_cb    
;    
;    ORG 0x3200
;
;can_read_rxbuf_cb    
    
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
    retfie
    
spi_interrupt
    bsf	    VISR, 0
    bcf	    PIR1, 3
    return
    
can_interrupt
    bsf	    VISR, 1
    bcf	    INTCON3, 1
    return

ioc_interrupt
    btfss   PORTC, 1
    bsf	    VISR, 2
    btfss   PORTC, 6
    bsf	    VISR, 3
    bcf	    INTCON, 0
    return
    
inter_low
    nop
    retfie
    
    END