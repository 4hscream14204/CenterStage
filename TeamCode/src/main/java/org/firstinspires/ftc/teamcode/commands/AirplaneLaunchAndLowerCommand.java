package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class AirplaneLaunchAndLowerCommand extends SequentialCommandGroup {

    public AirplaneLaunchAndLowerCommand(AirplaneLauncherSubsystem airplaneLauncherSubsystem, ArmSubsystem armSubsystemCon) {
        new InstantCommand(()->airplaneLauncherSubsystem.release());
        new WaitCommand(1000);
        new InstantCommand(()->airplaneLauncherSubsystem.lower());

    }
}
