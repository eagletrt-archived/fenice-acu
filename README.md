# fenice-sensors
Sensors Repository for Fenice

## Encoder Docs and Usage
### Encoder Specifics
- **LM13**  
    - **Interpolation factor** => 400   
    - **Resolution** => 5 um  
    - **Minimum edge separation** => 0.12us (8MHz)  
- **MR075E**  
    - **Outer Diameter** => 75.4 ± 0.1 mm  
    - **Inner Diameter** => 60 H7 mm  
    - **Circumference** => `3.1415926535 * 0.1005 = 0.236876086m` 
    - **Poles** => 120  
    - **cpr** => 48000  
    - **ppr** => 12000   
    - **Circumference with datasheet LM13 resolution** => `48000*0.000005 = 0.24m`  
- **Wheel**  
    - **Diameter** => 0.395 m
    - **Circumference** => `3.1415926535 * 0.395 = 1.2409290981325m`  
- **Speed**  
    - **Speed1:** is the speed calculated with the real circumference of the encoder `mult fact = 5.238726792`  
    - **Speed2:** is the speed calculated with the resolution and the cpr of the encoder `mult fact = 5.170416667`  
- **Price**  
    - ???
    

### How it works
The sensor is made from two components: 
- **The ring**: it is a circle made of magnetinc poles. It is made in a such  way, that by rotating it, you can read 2 signals _sin_ and _cos_ with an offset of 90°.
- **The Reader sensor**: it reads the magnetic signal from the ring and transform the signal from analogic to digital. Thanks to interpolation, it can give more resolution and be more prcise during the conversion. The digital signal can be distinguish in ***A*** and ***B*** and thanks to the offset of 90°, reading the signals you can determine both, direction and speed.
> **A** -011001100110
> **B** -001100110011


### Usage
#### Inputs
The inputs are 2 squared signals **A** and **B** from the LM13 sensor with a offset difference of 90°`A = B + 90° = B + 1/4*Period`.
#### Outputs
- **Encoder Speed**: Is the speed calculated with the encoder's diameter and signal inputs.
- **Wheel Speed**: Is the speed derived from the _Ecoder Speed_ calculated with both encoder's diameter and wheel's diameter.
- **Wheel Speed2**: Is the speed calculated with the _resolution_ given by the specifics of the sensors and the diameter of the wheel.
#### Process
- **Signals**: There are two interrupts on the GPIO receiving the signals _A_ and _B_. At every interrupt, of the signals, the __cp__ variable will be incremented, the __direction__ will be decided and will be memorized if the edge is __rising__ or __falling__.
- **Timers**: There are two timers to make the measure work properly.  
    - **TIM3**: This timer is giving the frequency for reading the speed. So every time the timer's period elapses, the second timer starts and the speed measurment begins.  
    ***Period***: 1s
    - **TIM4**: This timer gives the period in which the measurment is maked. In fact, the interrupts on the signals _A_ and _B_ are active only in this period.
    ***Period***: 0.1s 
        - **DO NOT EXCEED 9s** otherwise you will have an integer overflow
