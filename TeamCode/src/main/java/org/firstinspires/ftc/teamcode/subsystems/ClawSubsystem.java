package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ClawSubsystem extends SubsystemBase {

    private Servo srvClaw;
    private double dblClawOpen = 0.13;
    private double dblClawClose = 0;
    public RobotBase.ClawState clawState;

    public ClawSubsystem(Servo clawConstructor, boolean bolLeftSide) {
        srvClaw = clawConstructor;
        clawState = RobotBase.ClawState.OPEN;
        if (bolLeftSide != true) {
            dblClawOpen = 0.87;
            dblClawClose = 1;
        }
        clawOpen();
    }

    public void clawOpen() {
        srvClaw.setPosition(dblClawOpen);
        clawState = RobotBase.ClawState.OPEN;
    }

    public void clawClose() {
        srvClaw.setPosition(dblClawClose);
        clawState = RobotBase.ClawState.CLOSED;
    }

    public void clawToggle() {
        switch (clawState) {
            case OPEN:
                clawClose();
                break;
            case CLOSED:
                clawOpen();
                break;
        }
    }
}