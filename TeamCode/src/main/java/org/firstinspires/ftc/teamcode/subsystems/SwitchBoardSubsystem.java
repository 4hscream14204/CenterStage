package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class SwitchBoardSubsystem {

    TouchSensor tsButton;
    public SwitchBoardSubsystem(TouchSensor buttonConstructor) {
        tsButton = buttonConstructor;
    }

    public boolean getButtonState() {
        boolean bolButtonState;
        if(tsButton.isPressed()) {
            bolButtonState = true;
        } else {
            bolButtonState = false;
        }
        return bolButtonState;
    }
}
