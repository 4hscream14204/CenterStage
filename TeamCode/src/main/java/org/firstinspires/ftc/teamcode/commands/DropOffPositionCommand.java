package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DropOffPositionCommand extends SequentialCommandGroup {

    //CLOSE GRABBERS WHEN METHOD CALLED

    public DropOffPositionCommand(SlideSubsystem slideSubsystemCon,
                                  ArmSubsystem armSubsystemCon,
                                  WristSubsystem wristSubsystemCon,
                                  ClawSubsystem leftClawSubsystemCon,
                                  ClawSubsystem rightClawSubsystemCon,
                                  RobotBase.SlideHeight slideHeightCon) {
        if(leftClawSubsystemCon.clawState == RobotBase.ClawState.OPEN || rightClawSubsystemCon.clawState == RobotBase.ClawState.OPEN) {
            addCommands(
            new InstantCommand(()->leftClawSubsystemCon.clawClose()),
            new InstantCommand(()->rightClawSubsystemCon.clawClose()),
            new WaitCommand(500)
            );
        }
        addCommands(
                new InstantCommand(()->wristSubsystemCon.wristEscape()),
        new WaitCommand(500),
        new InstantCommand(()->armSubsystemCon.armDropOffPos()),
        new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop()),
        new InstantCommand(()->slideSubsystemCon.slideGoToPos(slideHeightCon)),
        new InstantCommand(()->wristSubsystemCon.wristDropOff())
        );
    }
}