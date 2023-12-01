package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class WristSubsystem extends SubsystemBase {

    Servo srvWrist;
    public RobotBase.WristState wristState;

    public WristSubsystem(Servo wristConstructor) {
        srvWrist = wristConstructor;
    }

    public void wristPickup() {
        wristState = RobotBase.WristState.GRABBING;
        srvWrist.setPosition(wristState.dblWristState);
    }

    public void wristEscape() {
        wristState = RobotBase.WristState.ESCAPE;
        srvWrist.setPosition(wristState.dblWristState);
    }

    public void wristDropOff() {
        wristState = RobotBase.WristState.DROPOFF;
        srvWrist.setPosition(wristState.dblWristState);
    }
}