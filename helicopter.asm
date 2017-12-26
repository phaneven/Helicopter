.include "m2560def.inc"

.def temp1 = r16
.def temp2 = r17
.def temp3 = r22
;;;;;;keypad register and parameter;;;;;;;;
.def row = r18 ; current row number
.def col = r19 ; current column number
.def rmask = r20 ; mask for current row during scan
.def cmask = r21 ; mask for current column during scan
.equ PORTFDIR = 0xF0 ; PF7-4: output, PF3-0, input
.equ INITCOLMASK = 0xEF ; scan from the leftmost column,
.equ INITROWMASK = 0x01 ; scan from the top row
.equ ROWMASK =0x0F ; for obtaining input from Port F
;;;;;;LCD parameter;;;;;;;;;;;;;;;;;;;;;;;
.equ LCD_CTRL_PORT = PORTA
.equ LCD_CTRL_DDR = DDRA
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ LCD_DATA_PORT = PORTK
.equ LCD_DATA_DDR = DDRK
.equ LCD_DATA_PIN = PINK
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.macro STORE
.if @0 > 63
sts @0, @1
.else
out @0, @1
.endif
.endmacro

.macro LOAD
.if @1 > 63
lds @0, @1
.else
in @0, @1
.endif
.endmacro
;;;;;;;;;;;;macro;;;;;;;;;;;;;;;;
.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	ldi r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro do_lcd_n
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro pressDelay  ; press delay for keypad 
	ldi r28, low(5000)
	ldi r29, high(5000)
	ldi r27, 170
loop: 
	adiw r29:r28, 1
	cpi r29, high(5000)
	breq done 
	rjmp loop 
done:
	clr r28
	clr r29
	dec r27
	brne loop	
.endmacro

.equ loop_count=124 ; press delay for one second
.macro oneSecondDelay
	ldi r28,low(loop_count)
	ldi r29,high(loop_count)
	clr r27
	clr r26
 loop:cp r26,r28
 	  cpc r27,r29
	  brsh done
	  call sleep_1ms
	  adiw r27:r26,1
	  nop 
	  rjmp loop
 done:
 	  nop
.endmacro
;;;;;;;;;;transfer value between data memory and register;;;;;;
.macro transfer_to_data ; transfer value of helicopter to dseg
	ldi XL, low(@1)
	ldi XH, high(@1)
	ldi temp1, @0
	st X,temp1
.endmacro
.macro transfer_to_register ; transfer value of helicopter to register
	ldi XL, low(@0)
	ldi XH, high(@0)
	ld @1, X
.endmacro
.macro move_to_data ; move value of helicopter to dseg
	ldi XL, low(@1)
	ldi XH, high(@1)
	mov temp1, @0
	st X,temp1
.endmacro


;;;;;;;;;;;;;;;;;;;;data memory and program memory;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.dseg
.org 0x200
posX: .byte 1 
posY: .byte 1 
posZ: .byte 1 
dir: .byte 1
speed: .byte 1
state_flag: .byte 1
hovering_flag: .byte 1
GlobalCounter: .byte 1
button_flag: .byte 1

flyingCounter: .byte 1
led_pattern:.byte 1
hole_counter: .byte 1
round_counter: .byte 1

.cseg
.org 0
	jmp RESET
.org INT0addr
	jmp EXT_INT0
.org INT1addr
	jmp EXT_INT1
.org INT2addr
	jmp EXT_INT2 ; opo
.org OVF0addr
	jmp Timer0OVF


	
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;External Interrupt;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
EXT_INT0: ;;;;;;;;;;;;;;;speed up;;;;;;;;;;;;;;;;;;;;;	
    transfer_to_register button_flag,temp3
	cpi temp3,0
	brne release
	transfer_to_data 1, button_flag
	transfer_to_register speed,temp2
	cpi temp2,4
	breq full_speed
	inc temp2
	call sleep_halfSecond ;;;;;;;;;;;;;;;wait 0.5 second
	move_to_data temp2,speed
	call flyingState
	reti
release: 
	reti
full_speed: reti

EXT_INT1: ;;;;;;;;;;;;;;speed down;;;;;;;;;;;;;;;;;;;
    transfer_to_register button_flag,temp3
	cpi temp3,0
	brne release1
	transfer_to_data 1, button_flag
	transfer_to_register speed,temp2
	cpi temp2,1
	breq zero_speed
	subi temp2,1
	call sleep_halfSecond ;;;;;;;;;;;;;;;wait 0.5 second
	move_to_data temp2,speed
	call flyingState
	reti
release1: 
	reti
zero_speed: reti

EXT_INT2:
	nop
	transfer_to_register hole_counter,temp2
	transfer_to_register round_counter,temp3
	inc temp2
	move_to_data temp2,hole_counter
	transfer_to_register hole_counter,temp2
    cpi temp2, 4
    brne not_One_Round
    transfer_to_data 0,hole_counter
    inc temp3
	move_to_data temp3,round_counter
not_One_Round:
    reti

Timer0OVF:
	adiw r25:r24,1
	cpi r24,low(1000)
  	brne NotSecond
  	ldi temp1,high(1000)
  	cp r25, temp1
  	brne NotSecond
	clr r25
	clr r24
	;;;;;;;;;;;;;;motor;;;;;;;;;;
	transfer_to_data 0,hole_counter
	transfer_to_data 0,round_counter
	call motor_speed
	;;;;;;;;;;;;;;;;;;;;;;;;
    transfer_to_data 0, button_flag;  set button flag
	transfer_to_register GlobalCounter, temp3
	inc temp3
	move_to_data temp3,GlobalCounter
    ;;;;;;;;led;;;;;;;;;
	transfer_to_register led_pattern, temp3
	out PORTC, temp3
	com temp3
	move_to_data temp3,led_pattern
	;;;;;;;;;;;;;;;;;;;;
	call position_change
	transfer_to_register speed,temp2
	cpi temp2,0
	breq remain
	transfer_to_register GlobalCounter,temp3
	move_to_data temp3,flyingCounter
	reti
NotSecond: reti
Remain:reti
    

RESET:
    clr temp3
    ;;;;initial helicopter;;;;;
	transfer_to_data 1,button_flag
	transfer_to_data 0,GlobalCounter
    transfer_to_data 25,posX
	transfer_to_data 25,posY
	transfer_to_data 0,posZ
	transfer_to_data 0,state_flag
	transfer_to_data 0,hovering_flag
	;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;Init PWM for motor control
	ldi r16,0b00001000
	sts DDRL,r16
	
	ldi r16,0xff
	sts OCR5AL,r16
	clr r16
	sts OCR5AH,r16
	
	ldi r16,(1<<CS50)|(1<<CS51)
	sts TCCR5B, r16
	ldi r16,(1<<WGM50)|(1<<COM5A1)|(1<<COM5A0)
	sts TCCR5A,r16
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;
    clr r16 ; keypad 
    ldi r16, PORTFDIR ; PF7:4/PF3:0, out/in
	out DDRF, r16

	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_CTRL_DDR, r16
	clr r16
	STORE LCD_DATA_PORT, r16
	STORE LCD_CTRL_PORT, r16

	do_lcd_command 0b00111000 ; 2x5x10
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x10
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x10
	do_lcd_command 0b00111000 ; 2x5x10
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

	do_lcd_data 'S'
	do_lcd_data 't'
	do_lcd_data 'a'
	do_lcd_data 'r'
	do_lcd_data 't'
	do_lcd_data ':'

	;enable external interrupt
	ldi r16,(2<<ISC00)|(2<<ISC10)
	sts EICRA,r16
	in r16,EIMSK
	ori r16,(1<<INT0)|(1<<INT1)
	out EIMSK,r16
	sei


	

main:
	ldi cmask, INITCOLMASK ; initial column mask
	clr col ; initial column
colloop:
	cpi col,4
	breq main; if all keys are scanned, repeat;
	out PORTF, cmask; otherwise, scan a column
	out PORTF, cmask
delay: 
	pressDelay
	in temp1, PINF ; read PORTF
	andi temp1, ROWMASK ; get the keypad output value
	cpi temp1, 0xF ; check if any row is low
	breq nextcol 	; if yes, find which row is low
	ldi rmask, INITROWMASK ; initialize for row check
	clr row ; 

rowloop:
	cpi row, 4
	breq nextcol ; the row scan is over.
	mov temp2, temp1
	and temp2, rmask ; check un-masked bit
	breq convert ; if bit is clear, the key is pressed
	inc row ; else move to the next row
	lsl rmask
	jmp rowloop

nextcol: ; if row scan is over
	lsl cmask
	inc col ; increase column value
	jmp colloop ; go to the next column
convert:
	cpi col,3
	breq main
	cpi row, 3					; if row is 3 we have a symbol or 0
	breq symbols
	mov temp1, row				; otherwise we have a number in 1-9
	lsl temp1
	add temp1, row				; temp1 = row * 3
	add temp1, col				; add the column address to get the value
	subi temp1,-1
	mov temp3, temp1
	call control_dir
	jmp main

symbols:
	cpi col, 0					; check if we have a star
	breq star
	cpi col, 2					; or if we have zero
	breq hash
	jmp main
star:
	transfer_to_register hovering_flag, temp3
	cpi temp3,1
	breq hovering_end
	transfer_to_data 1,hovering_flag
    call hovering_start
	jmp main
hovering_end:
	transfer_to_data 0,hovering_flag
	call flyingState
	jmp main
;;;;;;;# key;;;;;;;;;;;;;;;;;;;;;;;;;
hash:
	
	transfer_to_register state_flag, temp3
    cpi temp3,1
	breq landing
	;;;;;;;;;;;;;time counter;;;;;;;;
   	ldi temp3,0b0
	out TCCR0A,temp3
	ldi temp3,0b00000011
	out TCCR0B,temp3
	ldi temp3,1<<TOIE0
	sts TIMSK0,temp3
	clr temp3
	sei
	;;;;;;;;;;;;;;;;;;;;;;;;;;;
	transfer_to_data 1,state_flag ; display information about take off
	transfer_to_data 'U', dir
	transfer_to_data 2,speed
	call flyingState
	jmp main

landing:
	;;;;;;check hovering flag;;;;;;;;;;
	transfer_to_register hovering_flag, temp2
	cpi temp2,1
	breq stillHovering11
	;;;;;;;;;;;;;;;;
	transfer_to_data 0,state_flag
	transfer_to_data 'D', dir
	transfer_to_data 1,speed
	call flyingState
	jmp main
stillHovering11: jmp main; if hovering jump direct to main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.macro lcd_set
	sbi LCD_CTRL_PORT, @0
.endmacro
.macro lcd_clr
	cbi LCD_CTRL_PORT, @0
.endmacro

;
; Send a command to the LCD (temp1 = r16)
;

lcd_command:
	STORE LCD_DATA_PORT, r16
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret

lcd_data:
	STORE LCD_DATA_PORT, r16
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_DATA_PORT, r16
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	LOAD r16, LCD_DATA_PIN
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	STORE LCD_DATA_DDR, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret
sleep_20ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret
sleep_100ms:
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	ret
sleep_500ms:
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	ret
;;;;;wait half second;;;;;;;
sleep_halfSecond:
	ldi r31,high(500)
	ldi r30,low(500)
delayloop_halfSecond:
	call sleep_1ms
	sbiw r31:r30,1
	brne delayloop_halfSecond
	clr r31
	clr r30
	ret



;;;;;;;;;;;;;;;control direction;;;;;;;;;;;;;;;;;;;;
control_dir:
	;;;;;;;;;;;;;;;;
	transfer_to_register hovering_flag, temp2
	cpi temp2,1
	breq stillHovering12
	;;;;;;;;;;;;;;;;
	cpi temp3, 1
	breq UP_key
	cpi temp3, 3
	breq DOWN_key
	cpi temp3, 2
	breq FORWARD_key
	cpi temp3, 8
	breq BACKWARD_key
	cpi temp3, 4
	breq LEFT_key
	cpi temp3, 6
	breq RIGHT_key
	jmp main
stillHovering12:jmp main
UP_key:
	transfer_to_data 'U',dir
	call flyingState
	jmp main
DOWN_key:
	transfer_to_data 'D',dir
	call flyingState
	jmp main
FORWARD_key:
	transfer_to_data 'F',dir
	call flyingState
	jmp main
BACKWARD_key:
	transfer_to_data 'B',dir
	call flyingState
	jmp main
LEFT_key:
	transfer_to_data 'L',dir
	call flyingState
	jmp main
RIGHT_key:
	transfer_to_data 'R',dir
	call flyingState
	jmp main
ret
	

;;;;;;;;;;;;;;;;;;;;hovering;;;;;;;;;;;;;;;;;;;;;;;;
hovering_start:
	clr temp3
    do_lcd_command 0b00000001
    rcall sleep_1ms
    do_lcd_command 0b00000010			
    do_lcd_data 'p'
	do_lcd_data 'o'
	do_lcd_data 's'
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	do_lcd_command 0b00010100
	do_lcd_command 0b00010100
	do_lcd_data 'd'
	do_lcd_data 'i'
	do_lcd_data 'r'
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	do_lcd_command 0b00010100
	do_lcd_data 's'
	do_lcd_data 'p'
	do_lcd_data 'e'
	do_lcd_data 'e'
	do_lcd_data 'd'
	do_lcd_command 0b11000000
	rcall sleep_1ms
	do_lcd_data '('
	transfer_to_register posX, temp3
	call display_number
	do_lcd_data ','
	transfer_to_register posY, temp3
	call display_number
	do_lcd_data ','
	transfer_to_register posZ, temp3
	call display_number
	do_lcd_data ')'
ret
;;;;;;;;;;;;;;;;;;;;flying state;;;;;;;;;;;;;;;;;;;;;;;;;;;
flyingState:
    clr temp3
    do_lcd_command 0b00000001
    rcall sleep_1ms
    do_lcd_command 0b00000010			
    do_lcd_data 'p'
	do_lcd_data 'o'
	do_lcd_data 's'
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	do_lcd_command 0b00010100
	do_lcd_command 0b00010100
	do_lcd_data 'd'
	do_lcd_data 'i'
	do_lcd_data 'r'
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	do_lcd_command 0b00010100
	do_lcd_data 's'
	do_lcd_data 'p'
	do_lcd_data 'e'
	do_lcd_data 'e'
	do_lcd_data 'd'
	do_lcd_command 0b11000000 ; change line
	rcall sleep_1ms
	transfer_to_register posX, temp3
	call display_number
	do_lcd_data ','
	transfer_to_register posY, temp3
	call display_number
	do_lcd_data ','
	transfer_to_register posZ, temp3
	call display_number
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	transfer_to_register dir, temp3
	call display_direction
	do_lcd_command 0b00010100 ; shift cursor to left by one place
	transfer_to_register posZ, temp3
	cpi temp3,0
	breq continue_check ; if landed continue check 
	transfer_to_register speed, temp3
	call display_number
    do_lcd_data 'm'
	do_lcd_data '/'
	do_lcd_data 's'
	ret
continue_check:
	transfer_to_register state_flag, temp3 ; check whether the # key is pressed again
	cpi temp3,0
	breq safelyLand
	transfer_to_register speed, temp3
	call display_number
    do_lcd_data 'm'
	do_lcd_data '/'
	do_lcd_data 's'
	ret
safelyLand:
	transfer_to_data 0,speed
	transfer_to_register speed, temp3
	call display_number
    do_lcd_data 'm'
	do_lcd_data '/'
	do_lcd_data 's'
	ret

;;;;;;;;;;;;;;;;;;;display number;;;;;;;;;;;;;;;;;;;;;;;
display_number:
	clr temp2
	cpi temp3,10
    brge two_bit_number
	jmp one_bit_number
two_bit_number:
	subi temp3,10
	inc temp2
	cpi temp3,10
	brge two_bit_number
	subi temp2, -'0'
	do_lcd_n temp2
	clr temp2
	jmp one_bit_number
one_bit_number:
	subi temp3,-'0'
	do_lcd_n temp3
	clr temp2
ret
;;;;;;;;;;;;;;;;;;;;;display letter;;;;;;;;;;;;;;;;;;;;;;
display_direction:
	cpi temp3, 85; using ascii code to display letter
	breq UP
	cpi temp3, 68
	breq DOWN
	cpi temp3, 70
	breq FORWARD
	cpi temp3, 66
	breq BACKWARD
	cpi temp3, 76
	breq LEFT
	cpi temp3,82
	breq RIGHT
UP:
	do_lcd_data 'U'
	rjmp display_letter_finish
DOWN:
	do_lcd_data 'D'
	rjmp display_letter_finish
FORWARD:
	do_lcd_data 'F'
	rjmp display_letter_finish
BACKWARD:
	do_lcd_data 'B'
	rjmp display_letter_finish
LEFT:
	do_lcd_data 'L'
	rjmp display_letter_finish
RIGHT:
	do_lcd_data 'R'
	rjmp display_letter_finish
display_letter_finish:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;position change;;;;;;;;;;;;;;;;;;;

FORWARD_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne f_unchange
    transfer_to_register speed,temp3
	transfer_to_register posY,temp2
	add temp2,temp3
	cpi temp2,51; over bound?
	brge forward_flash
	move_to_data temp2,posY
    call flyingState
	ret
f_unchange:ret
forward_flash: ; display the last location and led start to flash
	transfer_to_data 50,posY
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret

BACKWARD_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne b_unchange
    transfer_to_register speed,temp3
	transfer_to_register posY,temp2
	cp temp2,temp3; over bound?
	brlo backward_flash
	sub temp2,temp3
	move_to_data temp2,posY
    call flyingState
	ret
b_unchange:ret
backward_flash:
	transfer_to_data 0,posY
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret

RIGHT_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne r_unchange
    transfer_to_register speed,temp3
	transfer_to_register posX,temp2
	add temp2,temp3
	cpi temp2,51; over bound?
	brge right_flash
	move_to_data temp2,posX
    call flyingState
	ret
r_unchange:ret
right_flash:
	transfer_to_data 50,posX
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret

LEFT_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne l_unchange
    transfer_to_register speed,temp3
	transfer_to_register posX,temp2
	cp temp2,temp3; over bound?
	brlo left_flash
	sub temp2,temp3
	move_to_data temp2,posX
    call flyingState
	ret
l_unchange:ret
left_flash:
	transfer_to_data 0,posX
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret

UP_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne u_unchange
    transfer_to_register speed,temp3
	transfer_to_register posZ,temp2
	add temp2,temp3
	cpi temp2,11; over bound?
	brge up_flash
	move_to_data temp2,posZ
    call flyingState
	ret
u_unchange:ret
up_flash:
	transfer_to_data 10,posZ
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret

DOWN_change_f:
	transfer_to_register hovering_flag,temp3
	cpi temp3,0
	brne d_unchange
    transfer_to_register speed,temp3
	transfer_to_register posZ,temp2
	transfer_to_register state_flag,temp1
	cp temp2,temp3; over bound?
	brlo down_flash
	add temp2,temp3
	add temp2,temp1
	cpi temp2,0
	breq landed; if safe landed jmp to landed
	transfer_to_register posZ,temp2
	sub temp2,temp3
	move_to_data temp2,posZ
    call flyingState
	ret
d_unchange:ret
down_flash:
	transfer_to_data 0,posZ
	transfer_to_data 0, speed
	call flyingState
	call led_bar
	ret
landed:
    do_lcd_command 0b00000001  ; clear the lcd
	do_lcd_command 0b00000010  ; back the blink to home 
	do_lcd_data 'D'
	do_lcd_data 'i'
	do_lcd_data 's'
	do_lcd_data 't'
	do_lcd_data 'a'
	do_lcd_data 'n'
	do_lcd_data 'c'
	do_lcd_data 'e'
	do_lcd_data ':'
	call calculate_distance
   	call display_number
	do_lcd_command 0b11000000
	do_lcd_data 'D'
	do_lcd_data 'u'
	do_lcd_data 'r'
	do_lcd_data 'a'
	do_lcd_data 't'
	do_lcd_data 'i'
	do_lcd_data 'o'
	do_lcd_data 'n'
	do_lcd_data ':'
	transfer_to_register flyingCounter,temp3
	inc temp3
	call display_number
	ret

position_change:
	transfer_to_register dir,temp3 ; transfer direction value to temp3
	cpi temp3,85
	breq UP_change
	cpi temp3,68
	breq DOWN_change
	cpi temp3,70
	breq FORWARD_change
	cpi temp3,66
	breq BACKWARD_change
	cpi temp3,76
	breq LEFT_change
	cpi temp3,82
	breq RIGHT_change
	ret
UP_change:
    call UP_change_f
	ret
DOWN_change:
    call DOWN_change_f
	ret
FORWARD_change:
    call FORWARD_change_f
	ret
BACKWARD_change:
    call BACKWARD_change_f
	ret
RIGHT_change:
    call RIGHT_change_f
	ret
LEFT_change:
    call LEFT_change_f
	ret

;;;;;;;;;;;;;;;calculate distance;;;;;;;;;;;

;;;;;;;;;;;;;;;calculate x position
calculate_distanceX: 
	transfer_to_register posX,temp3
	ldi temp2,25
	cpi temp3,25
	brlo minus_distanceX
	sub temp3,temp2
	ret
minus_distanceX:
	sub temp2,temp3
	mov temp3,temp2
	ret
;;;;;;;;;;;;;;;calculate y position
calculate_distanceY: 

	transfer_to_register posY,temp2
	ldi temp1,25
	cpi temp2,25
	brlo minus_distanceY
	sub temp2,temp1
	ret
minus_distanceY:
	sub temp1,temp2
	mov temp2,temp1
	ret
;;;;;;;;;;;;;;;calculate the full distance
calculate_distance:
	call calculate_distanceX
	call calculate_distanceY
	add temp3,temp2
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;display led bar;;;;;;;;;;
led_bar:
	ser temp1
	out DDRC,temp1
	ret
;;;;;;;;;;;;;;motor speed;;;;;;;;;;;;;
motor_speed:
	transfer_to_register speed,temp3
	cpi temp3,0 ;5 speed level
	breq speed0
	cpi temp3,1
	breq speed1
	cpi temp3,2
	breq speed2
	cpi temp3,3
	breq speed3
	cpi temp3,4
	breq speed4
	ret
speed0:
	ldi temp3, 0Xff
	sts OCR5AL,temp3
	clr temp3
	sts OCR5AH,r31
	ret
speed1:	
	ldi temp3, 0Xaf
	sts OCR5AL,temp3
	clr temp3
	sts OCR5AH,r31
	ret
speed2:	
	ldi temp3, 0X5f
	sts OCR5AL,temp3
	clr temp3
	sts OCR5AH,r31
	ret
speed3:	
	ldi temp3, 0X2f
	sts OCR5AL,temp3
	clr temp3
	sts OCR5AH,r31
	ret
speed4:	
	ldi temp3, 0X00
	sts OCR5AL,temp3
	clr temp3
	sts OCR5AH,r31
	ret
