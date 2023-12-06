package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class LowDropOffToggleCommand extends SequentialCommandGroup {

    public LowDropOffToggleCommand(SlideSubsystem slideSubsystemCon, ArmSubsystem armSubsystemCon, WristSubsystem wristSubsystemCon) {

        if(armSubsystemCon.getArmPosition() > 10/*Grabbing*/ || (slideSubsystemCon.slideHeight != RobotBase.SlideHeight.LOWEST && slideSubsystemCon.slideHeight !=RobotBase.SlideHeight.LOW)) {
            new InstantCommand(()->armSubsystemCon.armDropOffPos());
            new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
            new InstantCommand(()->slideSubsystemCon.slideGoToPos(RobotBase.SlideHeight.LOW));
            new InstantCommand(()->wristSubsystemCon.wristDropOff());
        } else {
            new UniversalGrabbingPosCommand();
        }
    }
}