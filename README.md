## The GPS Tracker Project

As I wanted to build experience in the field of embedded software programming, I was thinking of a nice project to work on. I found the idea of a GPS tracker. This project would allow me to interact with electronic components, develop in C for an MCU, gain experience with an embedded software IDE (STM32CubeIDE), test an embedded system and (last but not least) keep in touch with my love for motorcycles.

This project provides the C code required to turn an STM 32 MCU into a GPS tracker. The project notes are available [below](https://github.com/neo-knight-td/GPSTracker#notes-on-the-project). These notes summarize all the small obstacles I met while making this project. As I have only few hours a week to work on this, the notes are pretty useful to remember what I did last coding session and thus improve efficiency.

## Notes on the project

### On 4th of June 2023 :
1. USART 2 is redirected to ACM0.
2. ACM interface is working at 38400 bd, bits 8, no parity, 1 stop.
3. GPS data can be monitored in command shell console.
4. until now, gps data monitored is quite inconsistent :

```G3,23,70,8,21,087,3,10,844860$GNRMC,94,E,0.0072,K,A*6.86194,,04,09,
$GNGS
     $GPGSV,,31,06,61,09,68,GPGSV,4,42,22,3648
$GLG3,23,70,78,21,,84,63,20426.861,5036.486A

$GNGGA,,10,1.623,19,31,A,3,V,4,78,3,171,21,
                                           $GP1,20,4,13,468,
                                                            $GL31,,,8526.8522.,040
                                                                                  $GN,E,13,04,09,
                                                                                                 $GNF
$,04,13,07,10,177B
1,1070,178,21,0810,8.48605,N,183522.623,
2.53,,,,49,128*7,11,4,3,,21,            $GN,E,13,04.94.53,149,,28416,2,4,3,042,*450,2LGSV,83,3,,826.86190,A,23,,K,A*0042*7B
$G,6910,7,17328,8352A,50*6B
$G8618A,A.62,2.53,1.6,12,4,2868,PGSV31,2,13,,,686B
,17,,3,1L,5,A,A605,6A
GGA,,10,,09,
            $GN,1.93,3979
0,114,3,1,20,4,68,B
$9,00,84,6.47C
426.TG,,3.6.62,6,03$GNG1.9094V,4,6,17,16,GPGS67,00,11,25587,1,3,3,503,A*7426.8618N,0.155,0042
```

### On 6th of June 2023 :

5. The real TX of Pixhawk 4 GPS module is on the yellow cable. This needs to be connected to the 
PA02 pin of the stm32 board (UART TX).

6. Connection to USB 3.0 seems to perturb the location readout.

7. Exmaple of NEMA location obtained with USB 2.0 connection :
```
$GNGLL,5036.48727,N,00426.85867,E,144619.20,A,A*70
```

50°36.48727 N and 4°26.85867 E is almost the location of the testing location. Data seems consistent.

8. It seems impossible to debug session and receive serial data from the board at the same time.

### On 17th of June 2023 :

9. Readout of GPS data from ACM interface require some special configuration :
    
    * HSE & LSE in Crystal/Ceramic Resonator mode
    * Clock configure according to following schematic :

    ![plot](/Pictures/clock_config.png)

    Full config required is saved in [uart_example](/home/thomas/STM32CubeIDE/workspace_1/uart_example).

10. To transmit serial data to PC, follow the next steps :
    * Configure serial communication on PC with [Minicom](https://wiki.st.com/stm32mpu/wiki/How_to_get_Terminal)
    * Configure UART2 Tx in Asynchronous
    * Send data over UART2 in while loop :
```
  while (1)
  {
	//HAL_UART_Transmit(&huart2, UART2_rxBuffer, 12, 100);
	HAL_UART_Transmit(&huart2, (char*)"Hello World ! \r\n", 12, 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

11. From [here](https://electromaniaweb.wordpress.com/2019/10/20/nucleostm32f446re-teraterm-hello-world/) we learn that there exists a FWD from UART2_TX to STLINK. Which explains why when connecting Tx from Pixhawk GPS module we have a stream of NMEA data coming into ACM0 interface.

12. DMA should be configured in circular mode as explained [here](https://www.youtube.com/watch?v=chI8ZoOqOY4&t=684s).

13. Separating USART1 and USART 2 for receiving from Pixhawk (using circular DMA at 38400 bd) and for forwaring towards PC (using also 38400 bd) allowed for the first time to receive consistent data under 
``UART1_rxBuffer``. Associated code available on [GitHub Repository](https://github.com/neo-knight-td/GPSTracker/commit/4695cd7c1fd9cd82f45ec8f1316c37e5bc150a41).

### On 24th of June 2023 :

14. This [video](https://www.youtube.com/watch?v=ZIp05wYYuOs) shows an introduction to low power modes on STM32. Will definitly have to look at it in the future.

15. According to [NMEA](https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf) specifications, message of ID ``$GPGGA`` seems to contain all the required data (location, time, etc.). We will use it as the needle parameter in the ``strstr()`` function of our code.

16. Wrote necessary functions to save location string from nmea raw data in ``test.c``.

### On 25th of June 2023 :

17. Extract of location coordinates into ``DD.dd,DD.dd`` is working. Function ``strcpy()`` was stopping the program for unknown reason. Peformed the copy paste using a loop.

18. For integration of the module on the desired platform (motorbike), power consumption should be minimized. This [page](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_PWR#Introduction_to_the_low-power_modes) provides a guide to low power modes on STM32.
      * Certain modes require to understand what EXTI lines are. This [page](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_EXTI#:~:text=The%20EXTI%20(EXTernal%20Interrupt%2FEvent,%2C%20falling%2C%20or%20both).) provides a good explanation.
      * One solution would be to add a jumper between input signal source and another pin configured with an EXTI line.

### On 2nd of July 2023 :

19. Had to move working folder to default workspace because STM32CubeIDE did not want to start up.

20. Tried example on EXTI but callback never seem to be called (observed this already in the past).

22. Found in this [video](https://www.youtube.com/watch?v=jC_6FEpqzhA) that there actually exists a register callback.

### On the 3rd of July 2023 :

23. Found [here](https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2007%20-%20GPIO%20Interrupts%20(EXTI)%20using%20HAL%20(and%20FreeRTOS).pdf) that some function ``EXTI4_15_IRQHandler()`` is defined in another file and that this function gets called every time an event is triggered.

24. Found a tutorial [here](https://controllerstech.com/external-interrupt-using-registers/) for interrupt management with no HAL.

25. Found [here](https://stackoverflow.com/questions/58940598/how-to-use-exti-interrupt-with-hal-library-for-stm32f1) that the EXTI13 interrupt handler is missing.

26. Found [here](https://deepbluembedded.com/stm32-external-interrupt-example-lab/) a nice tutorial showing that ``HAL_EXTII_IRQHandler()`` should be defined in ``stm32g0xx_it.c``. When tracing back the declaration of ``HAL_GPIO_EXTI_IRQHandler()`` we realise 2 callback functions exist :
  * `` HAL_GPIO_EXTI_Rising_Callback() ``
  * `` HAL_GPIO_EXTI_Falling_Callback() ``

    In tutorial (see point 18, bullet 1) on EXTI, the callback provided was named ``HAL_GPIO_EXTI_Callback()`` which created the confusion. The newly discovered function callback was tested and found to be working.

### On the 13th of July 23

27. As the GPS module is probably consuming more current than a GPIO pin can sustain, a transistor (BN337) is used as a switch to power on the GPS module. A 3k3 Ohm resistor is put between the transistor base and STM 32 PIN PA10. The GPS module is connected between VDD and the transistor's collector. The emitter is connected to the ground. When user button is pressed, the GPS module is turned on (via the EXTI13 line) and the ``HAL_GPIO_EXTI_Rising_Callback()`` :

```
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
    //turn the GPS module on
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    //turn the led on (for debug purpose)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    //HAL_UART_Transmit(&huart2, (uint8_t*) loc_str, 50, 100);
  } else {
      __NOP();
  }
}
```
28. Cabling sheme to be created with fritzing.
29. ``-u _printf_float`` and ``-u _scanf_float`` to be added in Properties > C/C++ Build > Settings > Misc.

30. The GSM module blinks (1s) 7 to 9 times then shuts down for 2 seconds. It keeps booting up & shuting down. From sources found online, this would indicate an issue with the power supply. The current would be too low when trying to send a burst (2A are required at that moment). We need to re-organize the wiring and to purchase a battery.

31. ??
