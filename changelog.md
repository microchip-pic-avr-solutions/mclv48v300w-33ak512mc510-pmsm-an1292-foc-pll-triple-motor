# mclv48v300w-33ak512mc510-pmsm-an1292-foc-pll-triple-motor v1.0.2
### Release Highlights
- Minor edit to the ReadMe - added the interconnection diagram for triple motor control in the introduction section

# mclv48v300w-33ak512mc510-pmsm-an1292-foc-pll-triple-motor v1.0.1
### Release Highlights
- Minor bug fix to bootstrap
- Minor improvement to rotor lock

# mclv48v300w-33ak512mc510-pmsm-an1292-foc-pll-triple-motor v1.0.0
### Release Highlights
This is the first version of code for driving **three** Permanent Magnet Synchronous Motors (PMSM) with [dsPIC33AK512MC510](https://www.microchip.com/en-us/product/dsPIC33AK512MC510), using Sensorless Field Oriented Control (FOC) and PLL Estimator Algorithm. The demonstration is set up on the hardware platform [EV18H47A](https://www.microchip.com/en-us/development-tool/ev18h47a) "MCLV-48V-300W Development Board"  and [EV67N21A](https://www.microchip.com/en-us/development-tool/ev67n21a) "dsPIC33AK512MC510 Motor Control Dual In-line Module (DIM)".

The code is set up for running the following motors,
- 24V 3-Phase Brushless DC Motor - Hurst DMA0204024B101 [(AC300022)](https://www.microchip.com/en-us/development-tool/AC300022)
- 24V 3-Phase Brushless DC Motor - Hurst DMB0224C10002 [(AC300020)](https://www.microchip.com/en-us/development-tool/AC300020)
- 24V 3-Phase Leadshine Servo Motor [(ELVM6020V24FH-B25-HD)](https://www.leadshine.com/product-detail/ELVM6020V24FH-B25-HD.html)
- 24V 3-Phase ACT Brushless DC Motor [(ACT 57BLF01)](https://www.act-motor.com/brushless-dc-motor-57blf-product/)
- 24V 3-Phase ACT Brushless DC Motor [(ACT 57BLF02)](https://www.act-motor.com/brushless-dc-motor-57blf-product/)

All the motors are tested under no load conditions. To achieve optimal performance under loaded conditions, the control parameters in the firmware may need additional tuning.

### Features Added\Updated
- The first motor is run using the inverter stage available on the MCLV-48V-300W Development Board, as the board is designed to directly control only one three-phase motor
- To control the second and third motors, additional inverter card must be connected to the XPRO1 connector (J11) on the development board
- The application showcases the Sync PCI feature of PWM peripheral to synchronize the PWM signals for triple motor control
- Code flow is managed by State Machines to allow seamless integration of load-specific algorithms and application-specific communication interfaces
- Supports Dual-Shunt(phase shunts) current measurement schemes
- Integrates two variants of flux weakening,
    1. PI controller based on voltage circle limit (less dependent on motor parameters)
    2. Reference speed feed-forward control based on PMSM steady state equations(Id reference is computed from motor parameters); can be used when motor parameters are accurately known
- DC bus voltage compensation
- Assigned functions of push buttons and potentiometer in the firmware:
	1. Push Button #1(SW1) starts and stops motor. The Speed of the motor can be controlled using potentiometer(POT) in its entire speed range. Please note that the motor operates in the flux-weakening region beyond its nominal speed.
	2. Push Button #2(SW2) changes the direction of rotation
- Supports forward and reverse rotation of motor
- Improved start-up and closed loop transition
- Over Current Fault
	1. Software limit on the phase current
	2. DC Bus current protection using dsPIC® features such as PWM PCI, comparator and DAC
