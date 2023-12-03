package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmDropOffCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmDropOffCommand(ArmSubsystem armSubsystemCon) {
        this.armSubsystem = armSubsystemCon;
    }

    @Override
    public void initialize() {
        armSubsystem.armDropOffPos();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.armIsPassedSafeDrop();
    }
}
