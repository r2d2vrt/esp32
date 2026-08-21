            LIST      p=10F322
            #include  <P10F322.inc> 
            __CONFIG  (_FOSC_INTOSC& _BOREN_ON& _WDTE_SWDTEN& _PWRTE_OFF& _MCLRE_OFF& _CP_OFF& _LVP_OFF& _LPBOR_ON& _BORV_27& _WRT_OFF)
            errorlevel	-302		  ;Отключаем сообщения переключения банков  
 
cblock      0x40 
Reg_1                                 ; регистры для ПП задержек
Reg_2                                 ; -//-  
Reg_3                                 ; здесь храним уровень ШИМ1
Reg_4                                 ; здесь храним результат преобразования АЦП
Reg_5                                 ; в этом регистре храним данные количества замеров АЦП (если Uпит>Uуставки вводим константу количества замеров АЦП, если Uпит<Uуставки от константы -1)
endc 

; выход ШИМ1 вывод RA0
#define     button      PORTA,3       ; назначаем вывод для кнопки
#define     LED         PORTA,2       ; назначаем вывод для светодиода
#define     Ust_U       .85           ; устанавливаем уставку для контроля напряжения питания 86-1v ион 3,0 В 90-2,8 83-3,1
#define     CNT         .12           ; константа количества замеров АЦП (12 замеров, если Uпит<Uуставки, затем понижение яркости светодиода)

; старт программы
            org         0
            goto        START 

; инициализация контроллера
START       movlw       b'00000000'   ; отключаем выход осциллятора CLKR
            movwf       CLKRCON       ; -//-     
            movlw       b'00000000'   ; настраиваем осциллятор на 31 кГц (1 машинный цикл 129 мкСек)
            movwf       OSCCON        ; -//-     
            movlw       b'00000000'   ; глобальный запрет прерываний
            movwf       INTCON        ; -//-   
            movlw       b'00000000'   ; глобальный запрет периферийных прерываний
            movwf       PIE1          ; -//-
            movlw       b'00000000'   ; сбрасываем флаги регистра PIR1
            movwf       PIR1          ; -//-
            movlw       b'00001000'   ; настраиваем все порты на выход, RA3 на вход
            movwf       TRISA         ; -//-
            movlw       b'00000000'   ; отключаем аналоговые входы
            movwf       ANSELA        ; -//-
            movlw       b'00001000'   ; включаем подтягивающие резисторы PORTA, внутренний источник такта (FOSC/4), отключаем прескаллер Timer0
            movwf       OPTION_REG    ; -//-
            movlw       b'00001000'   ; включаем подтягивающий резистор на RA3
            movwf       WPUA          ; -//-	    
            movlw       b'00000000'   ; отключаем прерывания по переднему фронту PORTA
            movwf       IOCAP         ; -//-
            movlw       b'00001000'   ; включаем прерывания RA3 по заднему фронту PORTA
            movwf       IOCAN         ; -//-
            movlw       b'00000000'   ; отключаем ШИМ1
            movwf       PWM1CON       ; -//-
            movlw       b'00000000'   ; отключаем ШИМ2
            movwf       PWM2CON       ; -//-
            movlw       .4            ; период для ШИМ
            movwf       PR2           ; -//-
            movlw       b'00000100'   ; прескаллер таймера T2CON 1:1, включаем таймер T2CON (для ШИМ)
            movwf       T2CON         ; -//-
            clrf        PWM1DCL       ; очищаем регистры PWM1DCL и PWM1DCH (старший и младший регистр ввода уставок ШИМ1)	
            clrf        PWM1DCH       ; -//-   

            movlw       b'00000010'   ; Power-Save Sleep включен в режиме сна (для снижения потребления тока на 7 мкА)
            movwf       VREGCON       ; -//-
            movlw       b'00000000'   ; отключаем ИОН, отключаем температурный индикатор
            movwf       FVRCON        ; -//-
            movlw       b'00000000'   ; отключаем модуль АЦП
            movwf       ADCON         ; -//-

            movlw       b'00010111'   ; включаем сторожевой таймер, период 2 сек
            movwf       WDTCON        ; -//-

	        goto        Pow_off       ; уходим в сон, отключаем ШИМ
	
; основной цикл программы
go          clrwdt
            call        ADC           ; опрашиваем модуль АЦП, контроль напряжения питания
            btfsc       button        ; опрашиваем кнопку
            goto        go            ; закольцовка, если кнопка отжата
; обрабатываем данные с кнопки, если зафиксировано нажатие кнопки
            movlw       .249          ; вводим константы для ожидания отжатия кнопки (1s)
            movwf       Reg_1         ; -//- 
            movlw       .10           ; -//- 
            movwf       Reg_2         ; -//-
trc         decfsz      Reg_1, F
            goto        trc
            btfsc       button        ; ждем момента отжатия кнопки
            goto        LEDdown       ; уменьшаем яркость светодиода, если отжата кнопка менее чем за 1 сек
            clrwdt
            decfsz      Reg_2, F
            goto        trc
	        goto        Pow_off       ; уходим в сон, отключаем ШИМ, если кнопка нажата более чем 1 сек


LEDdown     decfsz      Reg_3, F      ; понижаем на 1 уровень ШИМ1
            goto        UST           ; ввод уставки в ШИМ1
            goto        Pow_off       ; отключаем устройство, если Reg_3 = 0

UST         movf        Reg_3,W       ; копируем в W содержимое Reg_3
            movwf       PWM1DCH       ; Reg_3 --> ШИМ
            call        Pause500ms
            goto        go


Pow_off     clrw                      ; 
            movwf       PWM1CON       ; отключаем ШИМ1
            movwf       FVRCON        ; отключаем ИОН, отключаем температурный индикатор
            movwf       ADCON         ; отключаем модуль АЦП
            clrf        PORTA         ; все выходы порта переводим в низкое состояние
            call        Pause500ms
            call        Pause500ms
            call        Pause500ms
            movlw       b'00001000'   ; разрешаем прерывание по изменению уровня PORTA
            movwf       INTCON        ; -//-   
            bcf         WDTCON,0      ; отключаем сторожевой таймер на период сна
            sleep
            clrf        IOCAF         ; сбрасываем флаг IOCAF (свидетельствующий о прерывании RA3)
            btfsc       button        ; опрашиваем кнопку
            goto        $-3           ; отправляем спать, если кнопка отжата
            bsf         WDTCON,0      ; включаем сторожевой таймер
            movlw       b'00000000'   ; глобальный запрет прерываний
            movwf       INTCON        ; -//-   
            movlw       b'11000000'   ; включаем ШИМ1
            movwf       PWM1CON       ; -//-
            movlw       .5            ; вводим уставку в ШИМ1 и производим запись в регистр хранения уставки ШИМ Reg_3
            movwf       PWM1DCH       ; -//-
            movwf       Reg_3         ; -//-
            movlw       b'10000001'   ; включаем ИОН 1,024V, отключаем температурный индикатор
            movwf       FVRCON        ; -//-
            movlw       b'00011101'   ; включаем модуль АЦП, RA1 вход, FOSC/2
            movwf       ADCON         ; -//-	    
            call        Pause500ms
            goto        go

 
ADC         bsf         ADCON,1       ; включаем преобразование АЦП
            btfsc       ADCON,1       ; ожидаем завершения
            goto        $-1           ; преобразования
            movfw       ADRES         ; перепишем результат преобразования
            movwf       Reg_4         ; в Reg_4
            bcf         STATUS, C     ; сбрасываем флаг C
            movlw       Ust_U         ; загружаем шаблон для проверки напряжения питания
            subwf       Reg_4, W      ; вычитаем Reg_4 из W
            btfss       STATUS, C     ; проверяем значение флага C, если =1 светодиод включаем, если =0 светодиод выключаем
            goto        led1          ; -//- 
            decfsz      Reg_5, F      ; -1 Reg_5
            return
            goto        led0          ; отключаем светодиод и понижаем уровень ШИМ, если Reg_5 = 0 (12 замеров АЦП равны)

led1        bsf         LED           ; включаем светодиод, если Uпит>Uуставки
            movlw       CNT           ; записываем константу в Reg_5 
            movwf       Reg_5         ; -//-
            return
led0        bcf         LED           ; выключаем светодиод, если Uпит<Uуставки
            goto        LEDdown



Pause500ms  movlw       .4
            movwf       Reg_1
            movlw       .6
            movwf       Reg_2
wr          decfsz      Reg_1, F
            goto        wr
            clrwdt
            decfsz      Reg_2, F
            goto        wr
            nop
            return

            end