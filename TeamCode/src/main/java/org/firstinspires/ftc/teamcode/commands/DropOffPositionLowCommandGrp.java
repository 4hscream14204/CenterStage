package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DropOffPositionLowCommandGrp extends SequentialCommandGroup {


    public DropOffPositionLowCommandGrp(SlideSubsystem slideSubsystemCon,
                                        ArmSubsystem armSubsystemCon,
                                        WristSubsystem wristSubsystemCon,
                                        IntakeSubsystem intakeSubsystemCon,
                                        RobotBase.SlideHeight slideHeightCon) {
        addCommands(
                new InstantCommand(()->intakeSubsystemCon.intake(-0.5)),
                new InstantCommand(()->armSubsystemCon.armDropOffLowestPos()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedWristSafe()),
                new InstantCommand(()->wristSubsystemCon.wristDropOff()),
                new InstantCommand(()->intakeSubsystemCon.intakeStop()),
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedExtendSlideSafe()),
                new ParallelCommandGroup(
                        new InstantCommand(()->armSubsystemCon.armSlowPower()),
                        new InstantCommand(()->slideSubsystemCon.slideGoToPos(slideHeightCon))
                )
        );
    }
}