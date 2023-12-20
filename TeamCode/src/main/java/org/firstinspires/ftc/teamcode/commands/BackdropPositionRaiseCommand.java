package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BackdropPositionRaiseCommand extends SequentialCommandGroup {

    RobotBase.SlideHeight newSlideHeight;

    public BackdropPositionRaiseCommand(SlideSubsystem slideSubsystemCon, WristSubsystem wristSubsystemCon, ArmSubsystem armSubsystemCon) {
        newSlideHeight = slideSubsystemCon.slideHeight;
        switch (slideSubsystemCon.slideHeight) {
            case LOWEST:
                newSlideHeight = RobotBase.SlideHeight.LOW;
                break;
            case LOW:
                newSlideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                break;
            case LOWMEDIUM:
                newSlideHeight = RobotBase.SlideHeight.MEDIUM;
                break;
            case MEDIUM:
                newSlideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                break;
            case MEDIUMHIGH:
                newSlideHeight = RobotBase.SlideHeight.HIGH;
                break;
            case HIGH:
                newSlideHeight = RobotBase.SlideHeight.HIGHEST;
                break;
        }
        new InstantCommand(()->armSubsystemCon.armDropOffPos());
        new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
        new InstantCommand(()->slideSubsystemCon.slideGoToPos(newSlideHeight));
        new InstantCommand(()->wristSubsystemCon.wristDropOff());
    }
}
