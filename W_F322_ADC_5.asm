; Программа выключателя двух нагрузок с контролем напряжения питания
; RA0 - Выход 2 (Нагрузка 2, включается после проверки VDD > 3.0В)
; RA1 - Выход пьезоизлучателя (пик)
; RA2 - Выход 1 (Нагрузка 1, включается первым)
; RA3 - Вход (кнопка на GND) - ТОЛЬКО для выключения
;
; Логика включения (автоматически при подаче питания):
; 1. Все выходы выключены
; 2. Через 1 сек: пик 0.3с, включение RA2
; 3. Через 1 сек (заряд): измерение VDD
; 4. Если VDD > 3.0В: пик 0.3с, включение RA0
; 5. Измерение VDD снова
; 6. Если VDD > 3.0В: пик 0.6с, переход в основной цикл
; 7. Если VDD <= 3.0В: отключить обе нагрузки, ПОСТОЯННЫЙ ПИК, зависнуть
;
; Основной цикл:
; VDD > 3.0В: RA0, RA2 - ON, RA1 - OFF
; 2.8В ? VDD < 3.0В: RA0, RA2 - ON, RA1 мигает раз в 30 сек
; 2.7В ? VDD < 2.8В: RA0, RA2 - ON, RA1 мигает раз в 5 сек
; VDD < 2.7В: пик 3 сек, отключение RA0 и RA2, зависнуть
; Кнопка > 1 сек: пик 1с, выключение обоих выходов, зависнуть
    
    LIST P=PIC10F322, R=DEC
    #include <p10f322.inc>
    
    ; ИЗМЕНЕНО: BORV_LO (1.9В) вместо BORV_27, чтобы МК работал ниже 2.7В
    __CONFIG _FOSC_INTOSC & _BOREN_ON & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _LVP_OFF & _LPBOR_ON & _BORV_LO & _WRT_OFF
    
    errorlevel -302

    cblock 0x40
        Reg_1
        Reg_2
        Reg_3           ; Счетчик циклов для RA1 (пик)
        Reg_4           ; Результат АЦП
        Button_Cnt      ; Счетчик нажатия кнопки
        Delay_Cnt       ; Счетчик для задержек
    endc

    #define Ust_3V0 .87
    #define Ust_2V8 .93
    #define Ust_2V7 .97     ; НОВЫЙ: порог для 2.7В
    #define button PORTA, 3

    org 0
    goto START

START:
    movlw b'00000000'   ; Генератор 31 кГц
    movwf OSCCON
    
    movlw b'00001000'   ; RA3=вход, RA0,RA1,RA2=выходы
    movwf TRISA
    
    movlw b'00000000'   ; Все пины цифровые
    movwf ANSELA
    
    clrf LATA           ; Все выходы выключены
    
    movlw b'00001000'   ; Подтяжка RA3 к VDD
    movwf WPUA
    
    movlw b'10000001'   ; FVR 1.024V включен
    movwf FVRCON
    
    movlw b'00011101'   ; АЦП: FVR канал, FRC тактирование
    movwf ADCON
    
    clrf Reg_3
    clrf Button_Cnt
    
    ; АВТОМАТИЧЕСКОЕ ВКЛЮЧЕНИЕ (без ожидания кнопки)
    goto TURN_ON_SEQUENCE

; =========================================================================
; ПОСЛЕДОВАТЕЛЬНОСТЬ ВКЛЮЧЕНИЯ С ПРОВЕРКАМИ НАПРЯЖЕНИЯ
; =========================================================================
TURN_ON_SEQUENCE:
    ; ШАГ 1: Задержка 1 секунда перед первым пиком (4 цикла * 0.3с = 1.2с)
    movlw .4
    movwf Delay_Cnt
DELAY_1S_1:
    call Pause300ms
    decfsz Delay_Cnt, F
    goto DELAY_1S_1
    
    ; ШАГ 2: Пик 0.3 сек и включение RA2 (Нагрузка 1)
    bsf LATA, 1
    call Pause300ms
    bcf LATA, 1
    
    bsf LATA, 2         ; Включаем Нагрузку 1
    
    ; ШАГ 3: Задержка 1 секунда для заряда конденсаторов (4 цикла)
    movlw .4
    movwf Delay_Cnt
DELAY_1S_2:
    call Pause300ms
    decfsz Delay_Cnt, F
    goto DELAY_1S_2
    
    ; ШАГ 4: Измерение напряжения АЦП
    call ADC
    
    ; Проверка: ADC < 87? (VDD > 3.0В)
    movf Reg_4, W
    sublw Ust_3V0
    btfsc STATUS, C
    goto VDD_OK_1       ; VDD > 3.0В - продолжаем
    
    ; VDD <= 3.0В после включения RA2 - отключить RA2 и зависнуть с постоянным пиком
    bcf LATA, 2
    bsf LATA, 1         ; Постоянный пик
    goto HANG           ; Зависнуть

VDD_OK_1:
    ; ШАГ 5: Повторный пик 0.3 сек и включение RA0 (Нагрузка 2)
    bsf LATA, 1
    call Pause300ms
    bcf LATA, 1
    
    bsf LATA, 0         ; Включаем Нагрузку 2
    
    ; ШАГ 6: Задержка 1 секунда для проверки напряжения (4 цикла)
    movlw .4
    movwf Delay_Cnt
DELAY_1S_3:
    call Pause300ms
    decfsz Delay_Cnt, F
    goto DELAY_1S_3
    
    ; ШАГ 7: Повторное измерение напряжения АЦП
    call ADC
    
    ; Проверка: ADC < 87? (VDD > 3.0В)
    movf Reg_4, W
    sublw Ust_3V0
    btfsc STATUS, C
    goto VDD_OK_2       ; VDD > 3.0В - продолжаем
    
    ; VDD <= 3.0В после включения RA0 - отключить ОБЕ нагрузки и зависнуть с постоянным пиком
    bcf LATA, 0         ; Отключить Нагрузку 2
    bcf LATA, 2         ; Отключить Нагрузку 1
    bsf LATA, 1         ; Постоянный пик
    goto HANG           ; Зависнуть

VDD_OK_2:
    ; ШАГ 8: Финальный пик 0.6 сек (2 цикла * 0.3с = 0.6с)
    bsf LATA, 1
    call Pause300ms
    call Pause300ms
    bcf LATA, 1
    
    clrf Reg_3          ; Сброс счетчика мигания
    goto MAIN

; =========================================================================
; ОСНОВНОЙ ЦИКЛ
; =========================================================================
MAIN:
    ; Проверяем кнопку на выключение
    btfss button        ; Если кнопка нажата (0)
    goto CHECK_OFF_BUTTON
    
    call ADC
    
    ; ИЗМЕНЕНО: Добавлена проверка VDD < 2.7В ПЕРЕД другими проверками
    movf Reg_4, W
    sublw Ust_2V7       ; 97 - ADC
    btfss STATUS, C     ; Если C=0 (ADC >= 97, т.е. VDD < 2.7В)
    goto VDD_LOW_CRITICAL ; Переход к критической обработке
    
    movf Reg_4, W
    sublw Ust_3V0
    btfsc STATUS, C
    goto VDD_HIGH
    
    movf Reg_4, W
    sublw Ust_2V8
    btfsc STATUS, C
    goto VDD_2V8_3V0
    
    goto VDD_2V7_2V8

; =========================================================================
; НОВЫЙ БЛОК: Критическое падение напряжения VDD < 2.7В
; =========================================================================
VDD_LOW_CRITICAL:
    ; Пик 3 секунды (10 циклов * 0.3с = 3.0с)
    bsf LATA, 1         ; Включить пик
    movlw .10
    movwf Delay_Cnt
CRITICAL_BEEP:
    call Pause300ms
    decfsz Delay_Cnt, F
    goto CRITICAL_BEEP
    
    bcf LATA, 1         ; Выключить пик
    
    ; Отключить обе нагрузки
    bcf LATA, 0         ; Нагрузка 2 OFF
    bcf LATA, 2         ; Нагрузка 1 OFF
    
    ; Зависнуть (не возвращаться к включению)
    goto HANG

VDD_HIGH:
    bsf LATA, 0
    bsf LATA, 2
    bcf LATA, 1
    clrf Reg_3
    goto DELAY_LOOP

VDD_2V8_3V0:
    bsf LATA, 0
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
    sublw .100
    btfsc STATUS, C
    goto BLINK_30_INC
    
    clrf Reg_3
    goto DELAY_LOOP
    
BLINK_30_INC:
    incf Reg_3, F
    goto DELAY_LOOP

VDD_2V7_2V8:
    bsf LATA, 0
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
    sublw .17
    btfsc STATUS, C
    goto BLINK_5_INC
    
    clrf Reg_3
    goto DELAY_LOOP
    
BLINK_5_INC:
    incf Reg_3, F
    goto DELAY_LOOP

; =========================================================================
; ПРОВЕРКА КНОПКИ ДЛЯ ВЫКЛЮЧЕНИЯ (более 1 секунды)
; =========================================================================
CHECK_OFF_BUTTON:
    clrf Button_Cnt
    
OFF_BUTTON_LOOP:
    call Pause300ms
    incf Button_Cnt, F
    
    btfsc button        ; Если кнопка отпущена
    goto MAIN           ; Возврат в основной цикл
    
    ; Проверка на удержание > 1 сек (4 цикла * 0.3с = 1.2с)
    movf Button_Cnt, W
    sublw .4
    btfsc STATUS, C
    goto OFF_BUTTON_LOOP
    
    ; Кнопка нажата > 1 сек - пик 1 сек и выключение
    bsf LATA, 1
    call Pause300ms     ; 0.3с
    call Pause300ms     ; 0.6с
    call Pause300ms     ; 0.9с
    call Pause300ms     ; 1.2с
    bcf LATA, 1
    
    bcf LATA, 0         ; Нагрузка 2 OFF
    bcf LATA, 2         ; Нагрузка 1 OFF
    
    ; Зависнуть (не возвращаться к началу)
    goto HANG

; =========================================================================
; БЕСКОНЕЧНЫЙ ЦИКЛ (зависание)
; =========================================================================
HANG:
    goto HANG           ; Бесконечный цикл

DELAY_LOOP:
    call Pause300ms
    goto MAIN

ADC:
    bsf ADCON, 1
    btfsc ADCON, 1
    goto $-1
    movfw ADRES
    movwf Reg_4
    return

Pause300ms:
    movlw .4
    movwf Reg_1
    movlw .4
    movwf Reg_2
wr:
    decfsz Reg_1, F
    goto wr
    decfsz Reg_2, F
    goto wr
    nop
    return

    end


