package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class GrabAndWristEscapeCommandGrp extends SequentialCommandGroup {

    public GrabAndWristEscapeCommandGrp(WristSubsystem leftWristSubsystemCon,
                                        WristSubsystem rightWristSubsystemCon,
                                        ClawSubsystem leftClawSubsystemCon,
                                        ClawSubsystem rightClawSubsystemCon,
                                        IntakeSubsystem intakeSubsystemCon) {

        if(leftClawSubsystemCon.clawState == RobotBase.ClawState.OPEN ||
                rightClawSubsystemCon.clawState == RobotBase.ClawState.OPEN) {
            addCommands(
                    new InstantCommand(()->leftClawSubsystemCon.clawClose()),
                    new InstantCommand(()->rightClawSubsystemCon.clawClose()),
                    new WaitCommand(500)
            );
        }
        if(leftWristSubsystemCon.wristState == RobotBase.WristState.GRABBING || rightWristSubsystemCon.wristState == RobotBase.WristState.GRABBING) {
            addCommands(
                    new InstantCommand(() -> leftWristSubsystemCon.wristEscape()),
                    new InstantCommand(() -> rightWristSubsystemCon.wristEscape()),
                    new WaitCommand(500)
            );
        }
    }
}
