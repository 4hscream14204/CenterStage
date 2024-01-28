package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DropOffPositionCommand extends SequentialCommandGroup {


    public DropOffPositionCommand(SlideSubsystem slideSubsystemCon,
                                  ArmSubsystem armSubsystemCon,
                                  WristSubsystem wristSubsystemCon,
                                  IntakeSubsystem intakeSubsystemCon,
                                  RobotBase.SlideHeight slideHeightCon) {
        addCommands(
                new InstantCommand(()->armSubsystemCon.armDropOffPos()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedWristSafe()),
                new InstantCommand(()->wristSubsystemCon.wristDropOff()),
                new InstantCommand(()->intakeSubsystemCon.intakeStop()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedExtendSlideSafe()),
                new InstantCommand(()->slideSubsystemCon.slideGoToPos(slideHeightCon))
        );
    }
}