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
        switch (slideSubsystemCon.slideHeight) {
            case LOWEST:
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.GRABBING));
                new UniversalGrabbingPosCommand();
                break;
            case LOW:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.LOWEST));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
            case LOWMEDIUM:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.LOW));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
            case MEDIUM:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.LOWMEDIUM));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
            case MEDIUMHIGH:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.MEDIUM));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
            case HIGH:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.MEDIUMHIGH));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
            case HIGHEST:
                new InstantCommand(()->armSubsystemCon.armDropOffPos());
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.HIGH));
                new InstantCommand(()->wristSubsystemCon.wristDropOff());
                break;
        }
    }
}