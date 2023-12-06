package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class MoveToDropoffCommand extends SequentialCommandGroup {
    //TO DO FIGURE OUT CORRECT MOTOR DIRECTIONS

    public MoveToDropoffCommand(SlideSubsystem slideSubsystemCon, ArmSubsystem armSubsystemCon, WristSubsystem wristSubsystemCon) {

        if(armSubsystemCon.getArmPosition() < 1/*Grabbing*/) {

            new WaitCommand(500);
            new WaitUntilCommand(()->armSubsystemCon.armIsPassedSafeDrop());
        }
    }
}