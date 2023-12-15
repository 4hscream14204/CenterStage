package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BackdropPositionRaiseCommand extends SequentialCommandGroup {

    RobotBase.SlideHeight slideHeight;

    public BackdropPositionRaiseCommand(SlideSubsystem slideSubsystemCon, WristSubsystem wristSubsystemCon, ArmSubsystem armSubsystemCon) {
        switch (slideSubsystemCon.slideHeight) {
            case LOWEST:
                slideHeight = RobotBase.SlideHeight.LOW;
                break;
            case LOW:
                slideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                break;
            case LOWMEDIUM:
                slideHeight = RobotBase.SlideHeight.MEDIUM;
                break;
            case MEDIUM:
                slideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                break;
            case MEDIUMHIGH:
                slideHeight = RobotBase.SlideHeight.HIGH;
                break;
            case HIGH:
                slideHeight = RobotBase.SlideHeight.HIGHEST;
                break;
        }
        new InstantCommand(()->armSubsystemCon.armDropOffPos());
        new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
        new InstantCommand(()->slideSubsystemCon.slideGoToPos(slideHeight));
        new InstantCommand(()->wristSubsystemCon.wristDropOff());
    }
}
