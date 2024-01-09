package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

@TeleOp (name= "Alliance Swap")
public class TeleAllianceSwap extends OpMode {

    private GamepadEx allianceController;
    public void init(){
        CommandScheduler.getInstance().reset();

        allianceController = new GamepadEx(gamepad1);

        allianceController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()->new ConditionalCommand(
                        new InstantCommand(()->DataStorageSubsystem.alliance = RobotBase.Alliance.BLUE),
                        new InstantCommand(()->DataStorageSubsystem.alliance = RobotBase.Alliance.RED),
                        ()->DataStorageSubsystem.alliance == RobotBase.Alliance.RED
                ));
    }
    public void loop(){

        telemetry.addData("Alliance", DataStorageSubsystem.alliance);
        CommandScheduler.getInstance().run();
    }
}
