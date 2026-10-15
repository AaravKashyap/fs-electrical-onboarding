//
// Created by Jackson Pinsonneault on 3/24/26.
//

#include "etc_controller.h"

// Assign appropriate GPIO objects depending on pin parameters
ETCController::ETCController(PinName APPS1_pin, PinName APPS2_pin): apps1(APPS1_pin), apps2(APPS2_pin) {
}

/* 
    Nice function to write which returns a wrapped version of the
    value given between 0 and 1 (-0.1 => 0, 10 => 1, 0.3 => 0.3).
*/
float ETCController::clamp(float value) {
    if (value < 0.0) {
        return 0.0;
    } else if (value > 1.0) {
        return 1.0;
    } else {
        return value;
    }

}

/*
    Simple function to write which returns a bool of true or false
    depending on whether low <= value <= high is true.
*/
bool ETCController::in_range(float value, float low, float high) {
    if (value >= low && value <= high) {
        return true;
    } else {
        return false;
    }
}

/*
    Complicated method with the goal of refreshing voltages,
    pedal positions, implausibilities, and motor torque demand.
*/
void ETCController::update_state() {
    /*
        Retrieve both APPS sensor voltages, determine pedal
        positions from globally set variables in header file, 
        then calculate a motor torque demand based off the 
        average if the two positions. 

        If you want an extra challenge, include the pedal
        deadzone percentage when determining pedal positions. 
        If you want to get really difficult, maybe even 
        include your own pedal mapping to make it non linear!
    */



    /*
        Finish the step listed above.
    */

    apps1_voltage = apps1.read() * 3.3;
    apps2_voltage = apps2.read() * 3.3;

    //It's to basically show if the position of the pedal is 0 or 1. It's the voltage value divided by the range of the voltage to give it into the clamp function and check if its closer to 0 (false) or 1 (true).
    apps1_position = clamp((((apps1_voltage - APPS1_MIN_VOLTAGE) / (APPS1_MAX_VOLTAGE - APPS1_MIN_VOLTAGE)) - PEDAL_DEADZONE_PERCENTAGE) / (1.0 - (2 * PEDAL_DEADZONE_PERCENTAGE)));
    apps2_position = clamp((((apps2_voltage - APPS2_MIN_VOLTAGE) / (APPS2_MAX_VOLTAGE - APPS2_MIN_VOLTAGE)) - PEDAL_DEADZONE_PERCENTAGE) / (1.0 - (2 * PEDAL_DEADZONE_PERCENTAGE)));

    // Here to simplify this function and readability.
    update_implausibilities();

    if (apps_implausibility || out_of_range_implausibility) {
        torque_demand = 0;
    } 
    
    else {
        torque_demand = ((apps1_position + apps2_position) / 2) * MAX_TORQUE;
    }
}

/*
    Timer based logic for the ETC to be rules compliant. MUST
    include rules T.4.2.4, T.4.2.9 (out of range).
*/
void ETCController::update_implausibilities() {
    /*
        Start of implaus logic checking.
    */
    float difference = abs(apps1_position - apps2_position);

    bool apps_mismatch = difference > 0.1;

    bool apps1_good = in_range (apps1_voltage, APPS1_MIN_VOLTAGE, APPS1_MAX_VOLTAGE);
    bool apps2_good = in_range (apps2_voltage, APPS2_MIN_VOLTAGE, APPS2_MAX_VOLTAGE);

    bool out_of_range = !apps1_good || !apps2_good;

    /*
        End of implaus logic checking.
    */

    /*
        Check how long each implaus has been active for and
        update the actual implaus result if it's been active 
        for too long.
    */

    if (apps_mismatch) {
        apps_implausibility_timer.start();
    }
    else {
        apps_implausibility_timer.stop();
        apps_implausibility_timer.reset();
    }

    if (out_of_range) {
            out_of_range_timer.start();
        }
    else {
        out_of_range_timer.stop();
        out_of_range_timer.reset();
    }

    if (chrono::duration_cast<chrono::milliseconds>(apps_implausibility_timer.elapsed_time()) > std::chrono::milliseconds(100)) {
        apps_implausibility = true;
    }
    if (chrono::duration_cast<chrono::milliseconds>(out_of_range_timer.elapsed_time()) > std::chrono::milliseconds(100)) {
        out_of_range_implausibility = true;
    }
}
