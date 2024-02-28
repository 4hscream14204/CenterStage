package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class WristSubsystem extends SubsystemBase {

    Servo srvWrist;
    private double dblWristGrabbing = 0.0427;//0.02
    private double dblWristDropOff = 0.87;
    private double dblWristEscape = 0.0827;
    private double dblWristDropOffLowest = 0.8561;//tune this number
    private double dblWristDropOffMH = 0.8961;
    public RobotBase.WristState wristState;

    public WristSubsystem(Servo wristConstructor, Boolean bolLeftSide) {
        srvWrist = wristConstructor;
        if (bolLeftSide != true) {
            dblWristGrabbing = 0.9722;
            dblWristDropOff = 0.1388;
            dblWristEscape = 0.9322;
            dblWristDropOffLowest = 0.1388;
            dblWristDropOffMH = 0.0922;
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

    public void wristDropOffMediumHigh() {
        wristState = RobotBase.WristState.DROPOFF;
        srvWrist.setPosition(dblWristDropOffMH);
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