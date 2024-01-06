package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class RaiseArmAndLauncherCommand extends SequentialCommandGroup {

    public RaiseArmAndLauncherCommand(AirplaneLauncherSubsystem airplaneLauncherSubsystemCon,
                                      ArmSubsystem armSubsystemCon,
                                      ClawSubsystem leftClawSubsystemCon,
                                      ClawSubsystem rightClawSubsystemCon) {
        addCommands(
                new InstantCommand(()-> leftClawSubsystemCon.clawClose()),
                new InstantCommand(()-> rightClawSubsystemCon.clawClose()),
                new InstantCommand(()-> armSubsystemCon.armDropOffPos()),
                new InstantCommand(()-> airplaneLauncherSubsystemCon.raise())
        );
    }
}
