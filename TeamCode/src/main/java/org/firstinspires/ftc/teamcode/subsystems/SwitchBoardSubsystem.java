package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class SwitchBoardSubsystem {

    TouchSensor redButton;
    TouchSensor greenButton;
    TouchSensor blackButton;
    TouchSensor blueButton;
    public SwitchBoardSubsystem(TouchSensor redButtonConstructor, TouchSensor blueButtonConstructor, TouchSensor blackButtonConstructor, TouchSensor greenButtonConstructor) {
        redButton = redButtonConstructor;
        greenButton = greenButtonConstructor;
        blackButton = blackButtonConstructor;
        blueButton = blueButtonConstructor;
    }
}
