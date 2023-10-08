## The GPS Tracker Project

As I wanted to build experience in the field of embedded software programming, I was thinking of a nice project to work on. I found the idea of a GPS tracker. This project would allow me to interact with electronic components, develop in C for an MCU, gain experience with an embedded software IDE (STM32CubeIDE), test an embedded system and (last but not least) keep in touch with my love for motorcycles.

This project provides the C code required to turn an STM 32 MCU into a GPS tracker. The project notes are available [below](#project_notes). These notes summarize all the small obstacles I met while making this project. As I have only few hours a week to work on this, the notes are also pretty useful to remember what I did last coding session and thus improve efficiency.

## <a name="project_notes"></a>Notes on the project

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
29. ``-u _printf_float`` and ``-u _scanf_float`` to be added in Project (Right Click) > Properties > C/C++ Build > Settings > Misc.

30. The GSM module blinks (1s) 7 to 9 times then shuts down for 2 seconds. It keeps booting up & shuting down. From sources found online, this would indicate an issue with the power supply. The current would be too low when trying to send a burst (2A are required at that moment). We need to re-organize the wiring and to purchase a battery.

### On the 9th of August 23

31. I finally received the material needed for going further in this project :
  * Step down converter
  * Balance charger
  * DC 3.7V 2100 mAh power supply
  * Minimum STM32 dev boards (STM32F103C8T6)
  * STM32 programmer dongle
  
  I charged the battery and soldered the default connections to some jumper wires. I connected the SIM 800 L GSM module to the battery and after a few seconds, the module's led blinks every 3s (meaning the connection to the network is established). This confirms the hypothesis made on point 30.

32. I took some time to realize it's no good idea to use blocking function inside an interrupt function. As an example, the following code :
```
while (1)
{
  //if flag is on, check sim 800 module state
  if (flag == 1){
    //transmit to pc we hit this block
    HAL_UART_Transmit(&huart2, (uint8_t*) sim800_test_str, 50, 100);
    //transmit check str to sim 800 module
    HAL_UART_Transmit(&huart3, (uint8_t*) sim800_test_str, 50, 100);
    //wait for reception
    HAL_UART_Receive(&huart3, (uint8_t*) UART3_rxBuffer, 700, 1000);
    //transmit status to PC
    HAL_UART_Transmit(&huart2, (uint8_t*) UART3_rxBuffer, 700, 100);

    //reset flag
    flag = 0;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
    //switch led on
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    //set flag on
    flag = 1;
  } else {
      __NOP();
  }
}
```

should be prefered above this code :

```
while (1)
{
  ...
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
    //switch led on
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    //transmit to pc we hit this block
    HAL_UART_Transmit(&huart2, (uint8_t*) sim800_test_str, 50, 100);
    //transmit check str to sim 800 module
    HAL_UART_Transmit(&huart3, (uint8_t*) sim800_test_str, 50, 100);
    //wait for reception
    HAL_UART_Receive(&huart3, (uint8_t*) UART3_rxBuffer, 700, 1000);
    //transmit status to PC
    HAL_UART_Transmit(&huart2, (uint8_t*) UART3_rxBuffer, 700, 100);
    
  } else {
      __NOP();
  }
}
```

This to prevent the controller to get stuck at some point in its time management.

33. I lost at least 3 hours to figure out that AT commands should be sent with the terminator ``\r\n`` and not with ``\n\r``. Once this was implemented, I could successfully interact with the gsm module (receive OK feedback).

34. I found [here](https://www.faranux.com/wp-content/uploads/2016/12/SIM800-Series_AT-Command-Manual_V1.09.pdf) a very useful document informing how to write AT commands.

35. I still can't send an sms with the commands sent. Here is what I obtain :

  * "AT;+CMGF=1\r\n" ==> OK after 5s
  * "AT;+CMGF=1;+CMGS=\"+32456413932\"\r\n" ==> ERROR

I found that I need to give the module some time to boot. I suspect faulty contacts at UART RX TX.

### On the 10th of August 2023

36. I borrowed code from [here](https://www.micropeta.com/video10), tweeked it a little (with extra delays) and got the module to send an sms.

### On the 11 of August 2023

37. I made a buch of functions for controlling the sim800 :
  * ```sim800_AT_OK(uint8_t debug_on)```
  * ```sim800_setup(char str_to_send[], uint8_t debug_on)```
  * ```sim800_send_sms(uint8_t debug_on)```
  * ```sim800_read_sms(uint8_t debug_on)```
  * ```sim800_delete_all_sms(uint8_t debug_on)```
  * ```sim800_originate_call(uint8_t debug_on)```

and one function to extract location from the gps module :
  * ```m8n_read_location(char nmea_raw_data[], char *loc_str[])```


  With these, I was able to complete a first version of the first stage of the project : the tracing capability. Whenever an sms is received by the sim800, the gps module is turned on and an sms is sent to my phone number. I am using the "RING" pin on the sim800 to detect an incoming sms. This pin was connected to the EXTI12 of the SIM32. 
  
  One issue is that the voltage sometimes drops inadvertently on pin RING (most probably due to the wrong pin connection, as it's not soldered yet). Therefore I am going to code a "false alert detection" feature that reads the last sms sent and if none exist or if last one dates of more than 15 seconds, detect the false alert and prevent sms from being sent.

  38. When reading all sms from sim800 :

  ```
  +CMGL: 1,"REC READ","+32456413932","","23/08/11,14:14:29+08"
Trace my bike

+CMGL: 2,"REC READ","+32456413932","","23/08/11,12:44:15+08"
Where is my bike ?

+CMGL: 3,"REC READ","+32456413932","","23/08/11,12:36:06+08"
Where is my bike ?

+CMGL: 4,"REC READ","+32456413932","","23/08/10,20:44:58+08"
Damn hell position 

+CMGL: 5,"REC READ","+32456413932","","23/08/11,13:46:18+08"
Where is my bike?

+CMGL: 6,"REC READ","+32456413932","","23/08/10,21:00:09+08"
Pos

+CMGL: 7,"REC READ","+32456413932","","23/08/11,14:18:59+08"
This is the last sms

+CMGL: 8,"REC READ","+32456413932","","23/08/11,14:21:05+08"
Bbb
```

39. 2 strange things :
  * I can't delete messages from the memory with the function ```sim800_delete_all_sms(uint8_t debug_on)```
  * Newly arrived sms are not always available when reading the unread messages : ```"AT+CMGL=\"REC UNREAD\"\r\n"``` 

### On the 15th of August 2023

40. When testing the SIM800 and after sending 10 "AT", SIM 800 was still not responding. Rebooted the STM and worked at first call. I suspect that the UART gets compromised at some point :

```
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
AT                                                                              
SIM800 : OK                                                                     
AT+CMGF=1                                                                       
AT+CPMS="SM","SM","SM"                                                          
AT                                                                              
SIM800 : OK                                                                     
AT+CMGL="ALL"                                                                   
AT+CMGL="ALL"                                                                   
+CMGL: 1,"REC UNREAD","+32456413932","","23/08/15,14:53:08+08"                  
LOCATE  
```
It seems afer sending and before reading an SMS, the UART line gets compromised. Therefore the UART3 is reset before these actions are performed.

### On the 20th of August 2023

41. Code was adapted to leave the GPS module on at all times so that when a location request is received, the response time is lowered. Also, the deletion of all messages from the memory is moved after the location is sent to the requestor. This with the aim to diminish the response time.

42. First functionnality is working quite well. Not all the code is written already but it's advanced enough to pass to the second part of the project : theft detection. This part involves an accelerometer. I found two old accelero in my drawer :
  * Ardafruit ADXL335 3-Axis Accelerometer with analog volt outputs
  * Red Bot Accelerometer Board (MMA8452Q)

I chose to go for the Red Bot Accelerometer Board as it's a digital sensor. The interesting thing about this board is also that communication is with I2C. That's a nice occasion to get started to this protocol (although I already learned about thanks to [this](https://www.allaboutcircuits.com/technical-articles/the-i2c-bus-hardware-implementation-details/) guide).

I followed [Digi-Key](https://www.digikey.be/en/maker/projects/getting-started-with-stm32-i2c-example/ba8c2bfef2024654b5dd10012425fa23)'s tutorial for the implementation of I2C sensor on the STM 32.

### On the 26th of August 2023

43. From the MMA8452Q documentation, we see read that I2C address of the accelerometer is either ```0x1C``` or ```0x1D```. After testing, ```0x1D``` is the good address.

46. In I2C, reading from the slave begins with a write (Ref. page 7 of [```slva704.pdf```](https://www.ti.com/lit/an/slva704/slva704.pdf?ts=1693003767721)).

47. I wrote two functions :
  * ```mma8452q_read_cfg(uint8_t debug_on)``` : reads the ELE & OAE bits from the configuration register and returns true if both of them are 1.
  * ```mma8452q_write_cfg(uint8_t bebug_on)``` : sets the ELE & OAE bits from the configuration register to 1

  When testing the functions, I could read and write to the register but when reading, the value was always 0, independently from the value written in the register. After double checking the documentation of the accelerometer, it appears that the modification of ```FF_MT_CFG``` (0x15) register content can only occur when device is in STANDBY mode... We thus need to create a function for switching from ACTIVE to STANDBY mode and vice versa.

48. After writing the function to transit from ACTIVE to STANDBY mode, I observe that even after the accelero is set in standby mode, the value read from the FF_MT_CFG register is not the one I wrote. Same thing with the SYSMOD register (the register used for switching between active and stanby modes). For this last one, it's explained from the documentation as the default val.. Omg, this register is read only !! :

49. What am I sometimes does not work

50. The standby and active functions seems to work. The result from mma8452q_read_sysmod seem erroneous.

### On the 2nd of September 2023

51. I ended up using HAL built in function for writing into and reading from the MMA8452Q registers : ```HAL_I2C_Mem_Write``` and ```HAL_I2C_Mem_Read```. With these two functions I was able to write a working suite of function for setting up the accelerometer.

### On the 2nd of September 2023

52. Now that all sensors required for the project are operational, the time has come to switch from the nucleo board (STM32G070RB) to the bluepill (STM32F103C8T6). I started with the gsm module that I connected on the bluepill.

  * First problem that I met is that no info was coming on the serial. I launched a degugging session and observed that the code was running properly. After some internet search, it turns out the bluepill (clones) do not come with a pre-installed bootloader which makes the micro usb connection inoperational for serial communication with the PC.
  * Second issue was that the sim800 module did not respond to the AT commands I sent to it. I deduced this was once again due to the bad connections of the jumpers at the level of then pin. I therefore decided to solder the pins to copper wires. I must have failed the soldering as, as from that moment, the sim800 could never connect to the network again.

53. Following the issues met in point 52, I made an additional Amazon order for :
  * 2 USB/TTL converters
  * 2 SIM800L modules
  * a kit of pcb (for building the prototype)

### On the 4th of September

54. Pending the delivery of my parcels, I started drawing the circuitry in fritzz.

### On the 9th of September

55. I received all my parcels and proceeded to the migration on the blue pill. After some troubleshooting, all three sensors are working. Now is the time to transpose the breadbaord circuitery onto the PCB.

### On the 14th of September

56. I am spening a lot of time on Fusion 360 for 3D printing the box... Not easy but interesting. Also, I ordered some capacitors and installed those at input of sim800 because battery had to be fully charged for the module to successfully connect to the network. If it was not the case the amount of current required was too high and the connection was never made.

### On the 29th of September

57. Now on the blue pill, when booting from the battery, the accelerometer does not respond after the sim800 connects to the network. The problem never occured when the module was plugged on USB (with the battery connected or not). I measured the voltages in both conditions - with and without battery - at the accelerometer input and obtained respectively 4.02 V and 3.2 V. First I found it weird that the mma8452q chip kept working well at 3.2 V despite the flag "5 V" on the VDD pin. Then I checked the datasheet and saw the chip was operating at 2.5 V. I supposed step down converter embedded to the accelerometer has a range of something like 3V - 5V. I thus tested connecting the module to the 3.3 V (and disconnected the module from the 5 V line) and saw it worked perfectly. There was a way to solve my issue. 

58. A small re-routing was made in the circuit. Instead of feeding the accelerometer with the 5 V line (which is shared with the sim 800), I isolated it from the rest of the circuit by connecting it to the 3.3 V. This made disappear the sporadic issue of accelerometer not responding once the sim800 had connected to the network (which I think was caused by a too high amount of current flowing from the 5 V line to the sim800 when the accelerometer was just booting up).

![plot](/Pictures/rerouting_note57.png)

59. I could not load establish the connection to the ST link after a softare update. This issues comes back and forth. Several solutions I found were :

* I followed this tutorial and it worked out the issue (after connecting the ST link to USB 3.0 port).

* I have to unplug battery for launching a debug session, otherwise target is not found. + need to be on USB 2.0 (not 3.0)

* Sometimes, I also need to unpower for a few seconds then power again.

### On the 3rd of October

57. Module seems not to wake up from stop mode (stuck at line 148). I did scrap the stop mode from the equation. Problem stays. Remaining possiblity is the EXTI that does not work.

When reveiwing the code, it seem function `` HAL_GPIO_EXTI_Falling_Callback() `` is nowhere defined. I looked back in these notes and tryed with `` HAL_GPIO_EXTI_Callback() `` (see point 26). This function is well defined. >> to be confirmed issue comes from there.

### On the 4th of October

58. I implemented the `` HAL_GPIO_EXTI_Callback() `` function but still it was never called. After browsing the internet a little, I found a similar case of some one who had forgot to check the box (enable NVIC). I checked that box and then it worked...

![plot](/Pictures/enable_nvic.png)

### On the 5th of October 

59. The GPS seemed not to be powered anymore, therefore I made a wiring check and identified a loose connection. Soldered it.

60. GPS signal seems way easier to catch when usb is plugged in

61. Led switches off after some time after send loc flag has been on without sending location. I tried troubleshoot without the TTL (only beased on led status) :
  * When in debug and that gps is locked, no issue, sms received.
  * When not in debug mode but with gps locked and usb plugged in, works once on two (not an approximation, really a cycle and on second time - when no send - led is lit for a short time only).

It was pretty difficult to explain the reason of these sms that were not sent. Therefore I connected the TTL to the module (soldered pins PA02 & PA03). Quickly, I saw I was getting an error following command ``AT+CMGL="ALL"  `` was issued to the sim800. I though a possible cause is that, when powering the module on usb, if battery was not plugged in soon enough, configuration was lost because sim800 turned off (no enough current) which leads to an error as the CMGL command requires a propoer config of the sim800 to work.

Do not forget to cut the line next time !!