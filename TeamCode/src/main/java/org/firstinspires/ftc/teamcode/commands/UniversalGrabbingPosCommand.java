package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class UniversalGrabbingPosCommand extends SequentialCommandGroup {

    public UniversalGrabbingPosCommand(RobotBase robotBase) {
        if(robotBase.leftClawSubsystem.clawState == RobotBase.ClawState.OPEN &&
                robotBase.rightClawSubsystem.clawState == RobotBase.ClawState.OPEN &&
                robotBase.armSubsystem.getArmPosition() > 20 &&
                robotBase.armSubsystem.armState == RobotBase.ArmState.DROPOFF
        ) {
            addCommands(
                    new InstantCommand(()->robotBase.armSubsystem.armReturning()),
                    new WaitCommand(700),
                    new InstantCommand(()->robotBase.leftSlideSubsystem.slideGoToPos(RobotBase.SlideHeight.GRABBING)),
                    new InstantCommand(()->robotBase.rightSlideSubsystem.slideGoToPos(RobotBase.SlideHeight.GRABBING)),
                    new InstantCommand(()->robotBase.leftWristSubsystem.wristPickup()),
                    new InstantCommand(()->robotBase.rightWristSubsystem.wristPickup()),
                    new WaitCommand(600),
                    new InstantCommand(()->robotBase.armSubsystem.armGrabbingPosition())
            );
        }
    }
}
