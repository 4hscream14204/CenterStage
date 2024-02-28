package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class GrabAndWristEscapeCommandGrp extends SequentialCommandGroup {

    public GrabAndWristEscapeCommandGrp(WristSubsystem wristSubsystemCon,
                                        ClawSubsystem clawSubsystemCon,
                                        ArmSubsystem armSubsystemCon
                                        ) {

        if(armSubsystemCon.armIsInGrabbing()) {
            if (clawSubsystemCon.clawState == RobotBase.ClawState.OPEN) {
                addCommands(
                        new WaitCommand(500),
                        new InstantCommand(() -> clawSubsystemCon.clawClose()),
                        new WaitCommand(500)
                );
            }
            if (wristSubsystemCon.wristState == RobotBase.WristState.GRABBING) {
                addCommands(
                        new InstantCommand(() -> wristSubsystemCon.wristEscape()),
                        new WaitCommand(200)
                );
            }
        }
    }
}
