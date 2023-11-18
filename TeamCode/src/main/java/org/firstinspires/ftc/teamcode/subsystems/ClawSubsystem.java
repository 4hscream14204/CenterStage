package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class ClawSubsystem extends SubsystemBase {

    private Servo srvLeftClaw;
    private Servo srvRightClaw;
    private final double dblClawOpen = 1;
    private final double dblClawClose = 0;
    private RobotBase.LeftClawState leftClawState;
    private RobotBase.RightClawState rightClawState;

    public ClawSubsystem(Servo leftClawConstructor, Servo rightClawConstructor) {
        srvLeftClaw = leftClawConstructor;
        srvRightClaw = rightClawConstructor;
    }

    public void leftOpen() {
        srvLeftClaw.setPosition(dblClawOpen);
        leftClawState = RobotBase.LeftClawState.OPEN;
    }

    public void leftClosed() {
        srvLeftClaw.setPosition(dblClawClose);
        leftClawState = RobotBase.LeftClawState.CLOSED;
    }

    public void rightOpen() {
        srvRightClaw.setPosition(dblClawOpen);
        rightClawState = RobotBase.RightClawState.OPEN;
    }

    public void rightClosed() {
        srvRightClaw.setPosition(dblClawClose);
        rightClawState = RobotBase.RightClawState.CLOSED;
    }

    public void leftClawToggle() {
        switch (leftClawState) {
            case OPEN:
                leftClosed();
                break;
            case CLOSED:
                leftOpen();
                break;
        }
        }
        public void rightClawToggle() {
        switch (rightClawState) {
            case OPEN:
                rightClosed();
                break;
            case CLOSED:
                rightOpen();
                break;
        }
        }
    }