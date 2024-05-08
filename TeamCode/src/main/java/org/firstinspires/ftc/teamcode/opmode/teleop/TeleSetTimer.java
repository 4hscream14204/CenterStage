package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

import java.sql.DataTruncation;

@TeleOp(name = "Set Timer")
public class TeleSetTimer extends OpMode {
    public int intTimerLength = DataStorageSubsystem.INTTIMERLENGTH;
    private GamepadEx timerController;
    public void init(){
        CommandScheduler.getInstance().reset();
        timerController = new GamepadEx(gamepad1);

        timerController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> intTimerLength = intTimerLength + 15000)
                        )
                );
        timerController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(
                                new InstantCommand(()-> intTimerLength = intTimerLength - 15000)
                        )
                );

    }
    public void loop(){

        telemetry.addData("Timer in seconds", intTimerLength / 1000);

        telemetry.update();

        CommandScheduler.getInstance().run();
    }

    public void stop(){
        DataStorageSubsystem.INTTIMERLENGTH = intTimerLength;
    }
}
