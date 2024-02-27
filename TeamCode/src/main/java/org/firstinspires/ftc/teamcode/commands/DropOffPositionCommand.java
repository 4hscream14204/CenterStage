package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
                new InstantCommand(()->intakeSubsystemCon.intake(-0.5))
        );
                if (slideHeightCon == RobotBase.SlideHeight.MEDIUMHIGH){
                    addCommands(
                            new InstantCommand(()->armSubsystemCon.armDropOffMHPos())
                    );
                } else if (slideHeightCon == RobotBase.SlideHeight.LOWEST) {
                    addCommands(
                            new InstantCommand(()->armSubsystemCon.armDropOffLowestPos())
                    );
                }else {
                    addCommands(
                            new InstantCommand(()->armSubsystemCon.armDropOffPos())
                    );
                }
                addCommands(
                new WaitUntilCommand(()->armSubsystemCon.armIsPassedWristSafe())
                );
                if (slideHeightCon == RobotBase.SlideHeight.MEDIUMHIGH) {
                    addCommands(
                            new InstantCommand(()->wristSubsystemCon.wristDropOffMediumHigh())
                    );
                } else if (slideHeightCon == RobotBase.SlideHeight.LOWEST) {
                    addCommands(
                            new InstantCommand(()->wristSubsystemCon.wristDropOffLowest())
                    );
                } else {
                    addCommands(
                            new InstantCommand(()->wristSubsystemCon.wristDropOff())
                    );
                }
                addCommands(
                        new InstantCommand(()->intakeSubsystemCon.intakeStop()),
                        new WaitUntilCommand(()->armSubsystemCon.armIsPassedExtendSlideSafe()),
                        new ParallelCommandGroup(
                                new InstantCommand(()->armSubsystemCon.armSlowPower()),
                                new InstantCommand(()->slideSubsystemCon.slideGoToPos(slideHeightCon))
                        )
                );
    }
}