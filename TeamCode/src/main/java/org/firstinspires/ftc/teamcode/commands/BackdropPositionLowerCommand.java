package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BackdropPositionLowerCommand extends SequentialCommandGroup {

    public BackdropPositionLowerCommand(SlideSubsystem slideSubsystemCon, WristSubsystem wristSubsystemCon, ArmSubsystem armSubsystemCon) {
        RobotBase.SlideHeight slideHeight;
        switch (slideSubsystemCon.slideHeight) {
            case LOW:
                slideHeight = RobotBase.SlideHeight.LOWEST;
                break;
            case LOWMEDIUM:
                slideHeight = RobotBase.SlideHeight.LOW;
                break;
            case MEDIUM:
                slideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                break;
            case MEDIUMHIGH:
                slideHeight = RobotBase.SlideHeight.MEDIUM;
                break;
            case HIGH:
                slideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                break;
            case HIGHEST:
                slideHeight = RobotBase.SlideHeight.HIGH;
                break;
        }
        new InstantCommand(()->armSubsystemCon.armDropOffPos());
        new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
        new InstantCommand(()->slideSubsystemCon.slideGoToPos());
        new InstantCommand(()->wristSubsystemCon.wristDropOff());
    }
}