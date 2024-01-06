package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class AirplaneLaunchAndLowerCommand extends SequentialCommandGroup {

    public AirplaneLaunchAndLowerCommand(AirplaneLauncherSubsystem airplaneLauncherSubsystem,
                                         ClawSubsystem leftClawSubsystemCon,
                                         ClawSubsystem rightClawSubsystemCon) {
        addCommands(
                new InstantCommand(()->airplaneLauncherSubsystem.release()),
                new WaitCommand(500),
                new InstantCommand(()->airplaneLauncherSubsystem.lower()),
                new WaitCommand(500),
                new InstantCommand(()->leftClawSubsystemCon.clawOpen()),
                new InstantCommand(()->rightClawSubsystemCon.clawOpen())
        );
    }
}
