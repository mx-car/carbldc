//
// Created by firat on 29.08.20.
//

#include "car/bldc/teensy32.h"

using namespace car::bldc;

/**
 * Configure Inhibit Pins as Output.
 * Done only once, called at Driver::initHardware
 * @param x - motor
 */

void Teensy32::initInhibitPins(Motor &x) {
    pinMode(x.inhibitPins.InhibitPinW, OUTPUT);
    pinMode(x.inhibitPins.InhibitPinU, OUTPUT);
    pinMode(x.inhibitPins.InhibitPinV, OUTPUT);

}

/**
 * Set Inhibit Pins HIGH.
 * Done only once on continuous SVPWM schemes , called at Driver::initHardware
 * @param x - motoor
 */
void Teensy32::activateInhibitPins(Motor &x) {
    digitalWriteFast(x.inhibitPins.InhibitPinW, HIGH);
    digitalWriteFast(x.inhibitPins.InhibitPinU, HIGH);
    digitalWriteFast(x.inhibitPins.InhibitPinV, HIGH);

}
/**
 * Set Inhibit Pins LOW.
 * Done only once on continuous SVPWM schemes , called at Driver::initHardware
 * @param x - motoor
 */
void Teensy32::deactivateInhibitPins(Motor &x) {
    digitalWriteFast(x.inhibitPins.InhibitPinW, LOW);
    digitalWriteFast(x.inhibitPins.InhibitPinU, LOW);
    digitalWriteFast(x.inhibitPins.InhibitPinV, LOW);
}

/**
 * Updates low level PWM registers with new duty cycle values.
 * Called every interrupt cycle
 * @param x - SVPWM duty cycles struct
 * @param motor - motor
 */
void Teensy32::updatePWMPinsDutyCycle(const SPWMDutyCycles &x, Motor &motor) {

    if (motor.initPins.InitPinW == 10 || motor.initPins.InitPinW == 22 || motor.initPins.InitPinW == 23) {

        FTM0_C3V = x.inDutyCycleW; //Teensy pin 10 -> FTM0_CH3pardom
        FTM0_C1V = x.inDutyCycleV;  // Teensy pin 22 (A8) -> FTM0_CH0
        FTM0_C0V = x.inDutyCycleU;  // Teensy pin 23 (A9) -> FTM0_CH1

    } else {

        FTM0_C7V = x.inDutyCycleV; //Teensy pin 5 -> FTM0_CH7
        FTM0_C4V = x.inDutyCycleU; //Teensy pin  6 -> FTM0_CH4
        FTM0_C2V = x.inDutyCycleW; // Teensy pin 9 -> FTM0_CH2
    }
    if(motor.inhibitor)  deactivateInhibitPins(motor);
    else activateInhibitPins(motor);
}


/**
 * Low level function for initializing PWM Pins.
 * - Center aligned PWM - Up Down Counter
 * - TOF interrupt
 * @TODO: try other interrupt options to have more time
 */
void Teensy32::initPWMPins() {
    FTM0_SC = 0; // required for other setup

    FTM0_CONF = 0xC0; //set up BDM in 11, FTM Counter functional -> 0000 1101 0000 0000
    FTM0_FMS = 0x00; //clear the WPEN (Write protection disabled) so that WPDIS is set in FTM0_MODE


    //FTM0_MODE|=0x05; // 0000 0101
    // This register contains the global enable bit for FTM-specific features and the control bits
    //used to configure:
    FTM0_MODE = 0b00000110; // 00000111

    //The Modulo register contains the modulo value for the FTM counter. After the FTM
    //counter reaches the modulo value, the overflow flag (TOF) becomes set at the next clock
    FTM0_MOD = (F_BUS / PWMFrequency) / 2;
    // FTM0_C6SC |= FTM_CSC_CHIE

    FTM0_C3SC = 0b00101000;
    FTM0_C3V = 0; //50%
    PORTC_PCR4 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 10 -> FTM0_CH3

    FTM0_C0SC = 0b00101000;
    FTM0_C0V = 0; //50%
    PORTC_PCR1 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 22 (A8) -> FTM0_CH0

    FTM0_C1SC = 0b00101000;
    FTM0_C1V = 0; //50%
    PORTC_PCR2 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 23 (A9) -> FTM0_CH1

    if (numberOfMotors > 1) {
        FTM0_C7SC = 0b00101000;
        FTM0_C7V = 0; //50%
        PORTD_PCR7 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 5 (A8) -> FTM0_CH7

        FTM0_C4SC = 0b00101000;
        FTM0_C4V = 0; //50%
        PORTD_PCR4 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 6 -> FTM0_CH4

        FTM0_C2SC = 0b00101000;
        FTM0_C2V = 0;
        PORTC_PCR3 |= PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Teensy pin 9 -> FTM0_CH2
    }

    FTM0_CNTIN = 0x00;


    FTM0_SC = 0b01101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000

    FTM0_SC = 0b01101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000 -> TOF interrupt disabled
    //FTM0_SC = 0b00101000; //CPWM MODE 0x48 EPWM -> 0x68 0110 1000


}

/**
 * Low level function to activate ADC peripheral
 * called once at Driver::initHardware
 */

void Teensy32::initADCconversions() {

    pinMode(ADC_PIN, INPUT);
    adc.adc0->setAveraging(4); // set number of averages
    adc.adc0->setResolution(12); // set bits of resolution
    adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
    adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
    adc.startContinuous(ADC_PIN);

}

/**
 * Simple function to get speed values from an Potentiometer
 * @return
 */
float Teensy32::setSpeedFromADC() {
    static float speed_cmd = 0;
    if (adc.adc0->isComplete()) {
        uint32_t value1 = adc.analogReadContinuous(ADC_PIN);
        speed_cmd = (value1 / static_cast<float_t > (adc.adc0->getMaxValue())) * 100;

    }
    return speed_cmd;

}
