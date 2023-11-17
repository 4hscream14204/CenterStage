package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private Servo srvLeftClaw;
    private Servo srvRightClaw;
    private final double dblClawOpen = 1;
    private final double dblClawClose = 0;

    public ClawSubsystem(Servo leftClawConstructor, Servo rightClawConstructor) {
        srvLeftClaw = leftClawConstructor;
        srvRightClaw = rightClawConstructor;
    }
        public void leftOpen() {
           srvLeftClaw.setPosition(dblClawOpen);
        }
        public void leftClosed() {
            srvLeftClaw.setPosition(dblClawClose);
        }
        public void rightOpen() {
            srvRightClaw.setPosition(dblClawOpen);
        }
        public void rightClosed() {
        srvRightClaw.setPosition(dblClawClose);
        }

    }

