package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    private Servo srvLeftClaw;
    private Servo srvRightClaw;

    public ClawSubsystem(Servo leftClawConstructor, Servo rightClawConstructor) {
        srvLeftClaw = leftClawConstructor;
        srvRightClaw = rightClawConstructor;
    }

}
