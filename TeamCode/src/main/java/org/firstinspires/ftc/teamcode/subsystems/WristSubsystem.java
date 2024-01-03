package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class WristSubsystem extends SubsystemBase {

    Servo srvWrist;
    private double dblWristGrabbing = 0.97;
    private double dblWristDropOff = 0.05;
    public RobotBase.WristState wristState;

    public WristSubsystem(Servo wristConstructor, Boolean bolLeftSide) {
        srvWrist = wristConstructor;
        if (bolLeftSide != true) {
            dblWristGrabbing = 0;
            dblWristDropOff = 0.9;
        }
        wristPickup();
    }

    public void wristPickup() {
        wristState = RobotBase.WristState.GRABBING;
        srvWrist.setPosition(dblWristGrabbing);
    }

    public void wristEscape() {
        wristState = RobotBase.WristState.ESCAPE;
        srvWrist.setPosition(wristState.dblWristState);
    }

    public void wristDropOff() {
        wristState = RobotBase.WristState.DROPOFF;
        srvWrist.setPosition(dblWristDropOff);
    }
}