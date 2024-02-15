package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class WristSubsystem extends SubsystemBase {

    Servo srvWrist;
    private double dblWristGrabbing = 0.05;
    private double dblWristDropOff = 0.88;
    private double dblWristEscape = 0.09;
    private double dblWristDropOffLowest = 0.85;
    public RobotBase.WristState wristState;

    public WristSubsystem(Servo wristConstructor, Boolean bolLeftSide) {
        srvWrist = wristConstructor;
        if (bolLeftSide != true) {
            dblWristGrabbing = 0.98;
            dblWristDropOff = 0.15;
            dblWristEscape = 0.94;
        }
        wristPickup();
    }

    public void wristPickup() {
        wristState = RobotBase.WristState.GRABBING;
        srvWrist.setPosition(dblWristGrabbing);
    }

    public void wristEscape() {
        wristState = RobotBase.WristState.ESCAPE;
        srvWrist.setPosition(dblWristEscape);
    }

    public void wristDropOff() {
        wristState = RobotBase.WristState.DROPOFF;
        srvWrist.setPosition(dblWristDropOff);
    }

    public void wristDropOffLowest() {
        wristState = RobotBase.WristState.DROPOFF;
        srvWrist.setPosition(dblWristDropOffLowest);
    }

    public boolean wristIsInEscape() {
        boolean bolWristEscape = false;
        if(wristState == RobotBase.WristState.ESCAPE) {
            bolWristEscape = true;
        } else {
            bolWristEscape = false;
        }
        return bolWristEscape;
    }
}