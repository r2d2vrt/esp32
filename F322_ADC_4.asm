;24 августа 2026 года
;программа выключателя нагрузки (RA2) по нажатию кнопки на RA3 (на GND) 
;более 0,3 секунд - включить, более 2-х секунд - выключить
;с контролем напряжения питания микроконтроллера (RA1 - пьезоизлучатель) 
;VDD > 3.0В (ADC < 87): RA2 - ON, RA1 - OFF
;2.8В ≤ VDD < 3.0В (87 ≤ ADC < 93): RA2 - ON, RA1 мигает раз в 30 секунд
;2.7В ≤ VDD < 2.8В (ADC ≥ 93): RA2 - ON, RA1 мигает раз в 5 секунд
;при VDD < 2.7В - срабатывает BOR и все отключено
    
     LIST P=PIC10F322, R=DEC
    #include <p10f322.inc>
    
    __CONFIG _FOSC_INTOSC & _BOREN_ON & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _LVP_OFF & _LPBOR_ON & _BORV_27 & _WRT_OFF
    
    errorlevel -302

    cblock 0x40
        Reg_1
        Reg_2
        Reg_3
        Reg_4
        Button_Cnt
        Device_State
    endc

    #define Ust_3V0 .87
    #define Ust_2V8 .93
    #define button PORTA, 3

    org 0
    goto START

START:
    movlw b'00000000'
    movwf OSCCON
    
    movlw b'00001001'
    movwf TRISA
    
    movlw b'00000000'
    movwf ANSELA
    
    clrf LATA
    
    movlw b'00001000'
    movwf WPUA
    
    movlw b'10000001'
    movwf FVRCON
    
    movlw b'00011101'
    movwf ADCON
    
    clrf Reg_3
    clrf Button_Cnt
    clrf Device_State
    
    goto WAIT_BUTTON

WAIT_BUTTON:
    bcf LATA, 1
    bcf LATA, 2
    
WAIT_LOOP:
    btfsc button
    goto WAIT_LOOP
    
    clrf Button_Cnt
    
BUTTON_PRESS:
    call Pause300ms
    incf Button_Cnt, F
    
    btfsc button
    goto BUTTON_RELEASED
    
    movf Button_Cnt, W
    sublw .7            ; ИЗМЕНЕНО: было .4, стало .7 (2 сек / 0.3 сек ≈ 7)
    btfsc STATUS, C
    goto BUTTON_PRESS
    
    clrf Device_State
    bcf LATA, 1
    bcf LATA, 2
    goto WAIT_BUTTON

BUTTON_RELEASED:
    movf Button_Cnt, W
    sublw .2            ; ИЗМЕНЕНО: было .1, стало .2 (0.5 сек / 0.3 сек ≈ 2)
    btfss STATUS, C
    goto TURN_ON
    
    goto WAIT_BUTTON

TURN_ON:
    movlw .1
    movwf Device_State
    
    bsf LATA, 1
    call Pause300ms     ; ИЗМЕНЕНО: было Pause500ms
    bcf LATA, 1         ; ИЗМЕНЕНО: был второй вызов Pause500ms, теперь один Pause300ms = 0.3с
    
    clrf Reg_3
    
    goto MAIN

MAIN:
    btfss button
    goto CHECK_OFF_BUTTON
    
    call ADC
    
    movf Reg_4, W
    sublw Ust_3V0
    btfsc STATUS, C
    goto VDD_HIGH
    
    movf Reg_4, W
    sublw Ust_2V8
    btfsc STATUS, C
    goto VDD_2V8_3V0
    
    goto VDD_2V7_2V8

VDD_HIGH:
    bsf LATA, 2
    bcf LATA, 1
    clrf Reg_3
    goto DELAY_LOOP

VDD_2V8_3V0:
    bsf LATA, 2
    
    movf Reg_3, W
    btfss STATUS, Z
    goto BLINK_30_PAUSE
    
    bsf LATA, 1
    movlw .1
    movwf Reg_3
    goto DELAY_LOOP
    
BLINK_30_PAUSE:
    bcf LATA, 1
    
    movf Reg_3, W
    sublw .100          ; ИЗМЕНЕНО: было .60, стало .100 (30 сек / 0.3 сек = 100)
    btfsc STATUS, C
    goto BLINK_30_INC
    
    clrf Reg_3
    goto DELAY_LOOP
    
BLINK_30_INC:
    incf Reg_3, F
    goto DELAY_LOOP

VDD_2V7_2V8:
    bsf LATA, 2
    
    movf Reg_3, W
    btfss STATUS, Z
    goto BLINK_5_PAUSE
    
    bsf LATA, 1
    movlw .1
    movwf Reg_3
    goto DELAY_LOOP
    
BLINK_5_PAUSE:
    bcf LATA, 1
    
    movf Reg_3, W
    sublw .17           ; ИЗМЕНЕНО: было .10, стало .17 (5 сек / 0.3 сек ≈ 17)
    btfsc STATUS, C
    goto BLINK_5_INC
    
    clrf Reg_3
    goto DELAY_LOOP
    
BLINK_5_INC:
    incf Reg_3, F
    goto DELAY_LOOP

CHECK_OFF_BUTTON:
    clrf Button_Cnt
    
OFF_BUTTON_LOOP:
    call Pause300ms
    incf Button_Cnt, F
    
    btfsc button
    goto OFF_BUTTON_RELEASED
    
    movf Button_Cnt, W
    sublw .7            ; ИЗМЕНЕНО: было .4, стало .7
    btfsc STATUS, C
    goto OFF_BUTTON_LOOP
    
    clrf Device_State
    bcf LATA, 1
    bcf LATA, 2
    goto WAIT_BUTTON

OFF_BUTTON_RELEASED:
    goto MAIN

DELAY_LOOP:
    call Pause300ms     ; ИЗМЕНЕНО: было Pause500ms
    goto MAIN

ADC:
    bsf ADCON, 1
    btfsc ADCON, 1
    goto $-1
    movfw ADRES
    movwf Reg_4
    return

Pause300ms:             ; ИЗМЕНЕНО: было Pause500ms
    movlw .4
    movwf Reg_1
    movlw .4            ; ИЗМЕНЕНО: было .6, стало .4
    movwf Reg_2
wr:
    decfsz Reg_1, F
    goto wr
    decfsz Reg_2, F
    goto wr
    nop
    return

    end