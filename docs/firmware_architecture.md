# Blue ESC Firmware Architecture

## Analog
### ADC 1
**Function:** Used for phase voltage measurements
- Modes
    - Input 1
    - Input 5
    - Input 14
- DMA request
    - DMA request: ADC 1
    - Direction: Peripheral to memory
    - Priority: LOW
- Interrupt
    - DMA 1 channel 1 global interrupt
    - ADC 1 and ADC 2 global interrupt
- Mode: Independent mode
- Enable regular conversions: Enable
- External trigger conversion source: Regular conversion launched by software
- Number of conversions: 3
- Conversion 1
    - Channel: Channel 1
    - Sampling time: 2.5 cycles
    - Offset number: No offset
- Conversion 2
    - Channel: Channel 5
    - Sampling time: 2.5 cycles
    - Offset number: No offset
- Conversion 3
    - Channel: Channel 4
    - Sampling time: 2.5 cycles
    - Offset number: No offset
### ADC 2
**Function:** Used for phase current measurements
- Modes
    - VOPAMP 2 Channel
    - VOPAMP 3 Channel
- DMA request
    - DMA request: ADC 2
    - Direction: Peripheral to memory
    - Priority: LOW
- Interrupt
    - DMA 1 channel 1 global interrupt
    - ADC 1 and ADC 2 global interrupt
- Mode: Independent mode
- Enable regular conversions: Enable
- External trigger conversion source: Timer 1 trigger out event
- External trigger conversion edge: Trigger detection on the rising edge
- Number of conversions: 3
- Conversion 1
    - Channel: Channel Vopamp 2
    - Sampling time: 2.5 cycles
    - Offset number: No offset
- Conversion 2
    - Channel: Channel Vopamp 3
    - Sampling time: 2.5 cycles
    - Offset number: No offset
### OPAMP 1
**Function:** Used for phase U current measurement
- Mode: PGA Internally connected IO0 BIAS
- Power mode: Normal
- PGA gain: 16 or -15
- User trimming: Disable
### OPAMP 2
**Function:** Used for phase V current measurement
- Mode: PGA Internally connected IO0 BIAS
- Power mode: Normal
- PGA gain: 16 or -15
- User trimming: Disable
### OPAMP 3
**Function:** Used for phase W current measurement
- Mode: PGA Internally connected IO0 BIAS
- Power mode: Normal
- PGA gain: 16 or -15
- User trimming: Disable


## Timers

### Timer 1

**Function:** Used for PWM generation
- Base frequency = 160 MHz (check **Figure 17. Clock tree**)
- Prescaler (PSC) = 0
- Auto Reload Register (ARR) = 4999
- Center aligned mode
- TIM1 update interrupt = ON

#### Channel 1
- PWM generation CH1 and CH1N

#### Channel 2
- PWM generation CH2 and CH2N

#### Channel 3
- PWM generation CH3 and CH3N


### Timer 4
**Function:**
- Combined channels = PWM Input on CH1
- Prescaler (PSC): 2
- Auto Reload Register (ARR): 65535

### Timer 6
**Function:**
- Prescaler(PSC): 15
- Auto Reload Register (ARR): 65535

## Conectivity
### FDCAN 1
**Function:**

### I2C 1
**Function:**

### USART 1
**Function:** Serial communication to host PC
- Frequency 1000000 Bits/s
- Word length: 8 Bits
- Parity: None
- Stop bits: 1
 