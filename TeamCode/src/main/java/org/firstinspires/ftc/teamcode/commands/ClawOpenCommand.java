package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawOpenCommand extends SequentialCommandGroup {

    public ClawOpenCommand(ArmSubsystem armSubsystemCon, ClawSubsystem clawSubsystemCon) {
        if(armSubsystemCon.getArmPosition() > 315) {
            addCommands(
            new InstantCommand(()->clawSubsystemCon.clawOpen())
            );
        }
        new ConditionalCommand(
                new InstantCommand(()->clawSubsystemCon.clawOpen()),
                new InstantCommand(),
                ()-> armSubsystemCon.armIsPassedSafeDrop()
        );
    }
}
