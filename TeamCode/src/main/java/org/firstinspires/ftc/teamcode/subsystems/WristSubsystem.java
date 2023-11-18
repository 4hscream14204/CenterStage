package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {

    Servo leftWrist;
    Servo rightWrist;
    private double dblPickup = 0;
    private double dblDropOff = 1;

    public WristSubsystem(Servo leftWristConstructor, Servo rightWristConstructor) {
        leftWrist = leftWristConstructor;
        rightWrist = rightWristConstructor;
    }

    public void leftPickup() {
        leftWrist.setPosition(dblPickup);
    }
    public void leftDropOff() {
        leftWrist.setPosition(dblDropOff);
    }
    public void rightPickup() {
        rightWrist.setPosition(dblPickup);
    }
    public void rightDropOff() {
        rightWrist.setPosition(dblDropOff);
    }
}