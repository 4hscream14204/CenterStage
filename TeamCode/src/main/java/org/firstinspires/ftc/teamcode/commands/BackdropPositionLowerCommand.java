package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BackdropPositionLowerCommand extends SequentialCommandGroup {

    RobotBase.SlideHeight newSlideHeight;

    public BackdropPositionLowerCommand(SlideSubsystem slideSubsystemCon,
                                        WristSubsystem wristSubsystemCon,
                                        ArmSubsystem armSubsystemCon) {
        newSlideHeight = slideSubsystemCon.slideHeight;
        switch (slideSubsystemCon.slideHeight) {
            case LOW:
                newSlideHeight = RobotBase.SlideHeight.LOWEST;
                break;
            case LOWMEDIUM:
                newSlideHeight = RobotBase.SlideHeight.LOW;
                break;
            case MEDIUM:
                newSlideHeight = RobotBase.SlideHeight.LOWMEDIUM;
                break;
                /*
            case MEDIUMHIGH:
                newSlideHeight = RobotBase.SlideHeight.MEDIUM;
                break;
            case HIGH:
                newSlideHeight = RobotBase.SlideHeight.MEDIUMHIGH;
                break;
            case HIGHEST:
                newSlideHeight = RobotBase.SlideHeight.HIGH;
                */
        }
        addCommands(
                new InstantCommand(()->armSubsystemCon.armDropOffPos()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedWristSafe()),
                new InstantCommand(()->wristSubsystemCon.wristDropOff()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedExtendSlideSafe()),
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(newSlideHeight))
        );
    }
}