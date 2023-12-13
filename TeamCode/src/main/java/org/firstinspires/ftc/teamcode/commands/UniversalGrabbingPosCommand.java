package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class UniversalGrabbingPosCommand extends SequentialCommandGroup {

    RobotBase robotBase;

    public UniversalGrabbingPosCommand() {
        if(robotBase.leftClawSubsystem.clawState == RobotBase.ClawState.OPEN && robotBase.rightClawSubsystem.clawState == RobotBase.ClawState.OPEN) {
            new InstantCommand(()->robotBase.leftSlideSubsystem.slideGoToPos(RobotBase.SlideHeight.GRABBING));
            new InstantCommand(()->robotBase.rightSlideSubsystem.slideGoToPos(RobotBase.SlideHeight.GRABBING));
            new InstantCommand(()->robotBase.leftWristSubsystem.wristPickup());
            new InstantCommand(()->robotBase.rightWristSubsystem.wristPickup());
            new WaitCommand(1000);
            new InstantCommand(()->robotBase.armSubsystem.armGrabbingPosition());
            new InstantCommand(()->robotBase.leftClawSubsystem.clawOpen());
            new InstantCommand(()->robotBase.rightClawSubsystem.clawOpen());
        }
    }
}
