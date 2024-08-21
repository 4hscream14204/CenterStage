package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class DropOffSequentialCommandGrp extends SequentialCommandGroup {

    public DropOffSequentialCommandGrp(RobotBase robotBase, RobotBase.SlideHeight slideHeightCon) {
        addCommands(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new GrabAndWristEscapeCommandGrp(
                                robotBase.leftWristSubsystem,
                                robotBase.leftClawSubsystem, robotBase.armSubsystem),
                        new GrabAndWristEscapeCommandGrp(
                                robotBase.rightWristSubsystem,
                                robotBase.rightClawSubsystem, robotBase.armSubsystem)
                ),
                new ParallelCommandGroup(
                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.leftWristSubsystem,
                                robotBase.intakeSubsystem,
                                slideHeightCon),
                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.rightWristSubsystem,
                                robotBase.intakeSubsystem,
                                slideHeightCon))
        ));


    }
}
