//
// Created by firat on 29.08.20.
//

#ifndef INC_2MOTORNEWBOARD_SVPWM_LUT_H
#define INC_2MOTORNEWBOARD_SVPWM_LUT_H

#include <Arduino.h>
#include <array>


struct ModulationIndexScalingParams{
    constexpr ModulationIndexScalingParams(float m, float c): offsetParam_m(m), offsetParam_c(c){};
    float offsetParam_m;
    float offsetParam_c;

};

class LUTGenerator{

public:
    static constexpr uint8_t motorPoleCount = 11;
    static constexpr uint16_t LUTSize = ENCODER_RESOLUTION / motorPoleCount;
    static constexpr uint16_t MAX_DUTY_CYCLE = (F_BUS / SVPWM_FREQUENCY) / 2;

    /**
     * Generates the Look Up Table for PWM duty cycles
     * CalculateDutyCycleW can be re-implemented for other SVPWM schemes such as symmetric, discontinuous etc.
     * - constexpr function, called once at compile time
     * @return
     */
    constexpr static  std::array<uint16_t, LUTSize> generate() {

        std::array <uint16_t, LUTSize> temp_table{};

        float modulation_index = 1.1547005f;

        uint angle = 0;
        for (int i = 0; i < LUTSize; i++) {

            temp_table[i] = static_cast<uint16_t>(calculateDutyCycleW(angle,modulation_index,MAX_DUTY_CYCLE));

            angle++;
        }

        return temp_table;
    }

    /**
     *
     * @param rotor_position - scaled (modulo'ed) encoder value that represents rotor flux. This is then modified to
     * a degree equivalent.
     * @param modulation_index - determines the power transferred to the Motor. Right now all tables are generated for
     * max power and then scaled down accordingly when needed.
     *
     * @param MAX_DUTY_CYCLE - required for calculation
     * @return The duty cycle of one phase
     */
    constexpr static uint16_t calculateDutyCycleW(uint16_t rotor_position,float modulation_index,uint16_t MAX_DUTY_CYCLE){
        constexpr float angleScaler = 360.0f / LUTSize;
        constexpr float deg_to_radians = 0.017453f;
        uint16_t IN_W_duty_cycle=0;

        float r1 = 0;
        float r2 = 0;
        float t7 = 0;
        float t0 = 0;
        float t1 = 0;
        float t2 = 0;
        float angle = static_cast<float>(rotor_position) * angleScaler;

        if (angle >= 0 && angle < 60) {

            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - angle) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) *
                 (1 - modulation_index * cos(angle * deg_to_radians) + (modulation_index / 6) * cos(3 * angle * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = (t2 + t7 + t1);
            //IN_U_duty_cycle = t2 + t7;
            //IN_V_duty_cycle = t7;

        } else if (angle >= 60 && angle < 120) {
            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - (angle - 60)) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle - 60) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) * (1 - modulation_index * cos((angle - 60) * deg_to_radians) +
                                         (modulation_index / 6) * cos(3 * (angle - 60) * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = t1 + t0;
            //IN_U_duty_cycle = t1 + t2 + t0;
            //IN_V_duty_cycle = t0;

        } else if (angle >= 120 && angle < 180) {

            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - (angle - 120)) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle - 120) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) * (1 - modulation_index * cos((angle - 120) * deg_to_radians) +
                                         (modulation_index / 6) * cos(3 * (angle - 120) * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = t7;
            //IN_U_duty_cycle = t1 + t2 + t7;
            //IN_V_duty_cycle = t2 + t7;


        } else if (angle >= 180 && angle < 240) {
            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - (angle - 180)) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle - 180) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) * (1 - modulation_index * cos((angle - 180) * deg_to_radians) +
                                         (modulation_index / 6) * cos(3 * (angle - 180) * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = t0;
            //IN_U_duty_cycle = t1 + t0;
            //IN_V_duty_cycle = t1 + t2 + t0;

        } else if (angle >= 240 && angle < 300) {
            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - (angle - 240)) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle - 240) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) * (1 - modulation_index * cos((angle - 240) * deg_to_radians) +
                                         (modulation_index / 6) * cos(3 * (angle - 240) * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = t2 + t7;
            //IN_U_duty_cycle = t7;
            //IN_V_duty_cycle = t1 + t2 + t7;


        } else if (angle >= 300 && angle < 360) {

            r1 = (sqrt(3) / 2) * modulation_index * sin((60 - (angle - 300)) * deg_to_radians);
            r2 = (sqrt(3) / 2) * modulation_index * sin((angle - 300) * deg_to_radians);

            t1 = r1 * (MAX_DUTY_CYCLE);
            t2 = r2 * (MAX_DUTY_CYCLE);
            t0 = (MAX_DUTY_CYCLE / 2) * (1 - modulation_index * cos((angle - 300) * deg_to_radians) +
                                         (modulation_index / 6) * cos(3 * (angle - 300) * deg_to_radians));
            t7 = MAX_DUTY_CYCLE - (t1 + t2 + t0);

            IN_W_duty_cycle = t1 + t2 + t0;
            //IN_U_duty_cycle = t0;
            //IN_V_duty_cycle = t1 + t0;


        }
        return IN_W_duty_cycle;


    }

    /***
     * LUT PWM values are calculated for maximum modulation index (max power). Due to complex nature of SVPWM,
     * a simple scaling doesnt work. Simply multiplying the full duty cycle with a scalar (0.5 for example) doesnt procude
     * the correct duty cycles. After this multiplication an offset need to be added to the scaled down duty cycle
     * This offset is a linear affine function of the scalar. The parameters of this functions are calculated here.
     * - constexpr function, called at compile time once.
     * @return
     */
    static  constexpr ModulationIndexScalingParams calculateModulationIndexScalingOffsetParameters() {

        uint16_t rotor_position = 200;
        float modulation_index_max = 1.1547005f;
        float IN_W_duty_cycle_max = calculateDutyCycleW(rotor_position, modulation_index_max, MAX_DUTY_CYCLE);

        float scalar1 = 0.4f;
        float modulationIndexNew = scalar1 * modulation_index_max;
        float dutyCycleScaledWithModuloIndex1 = IN_W_duty_cycle_max * scalar1;
        float dutyCycleCalculatedWithModuloIndex1 = calculateDutyCycleW(rotor_position, modulationIndexNew,
                                                                        MAX_DUTY_CYCLE);

        float scalar2 = 0.9f;
        modulationIndexNew = scalar2* modulation_index_max;
        float dutyCycleScaledWithModuloIndex2 = IN_W_duty_cycle_max * scalar2;
        float dutyCycleCalculatedWithModuloIndex2 = calculateDutyCycleW(rotor_position, modulationIndexNew,
                                                                        MAX_DUTY_CYCLE);


        float yy1 = dutyCycleCalculatedWithModuloIndex1 - dutyCycleScaledWithModuloIndex1;
        float yy2 = dutyCycleCalculatedWithModuloIndex2 - dutyCycleScaledWithModuloIndex2;

        float xx1 =  scalar1*100;
        float xx2 =  scalar2*100;

        float m = (yy2 - yy1) / (xx2 - xx1);

        float c = yy2 - (m * xx2);


        ModulationIndexScalingParams params{m, c};

        /*Serial.print(", c = ");
        Serial.print(static_cast<int16_t >(params.offsetParam_c));
        Serial.print(", m = ");
        Serial.println(static_cast<int16_t >(floorf(params.offsetParam_m )));
*/


        return params;

    }




};
#endif //INC_2MOTORNEWBOARD_SVPWM_LUT_H
