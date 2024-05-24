package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TimerSubsystem;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;

@TeleOp(name = "DuckHunt")
public class TeleDuckHunt extends OpMode {

    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public TimerSubsystem timerSubsystem;

    public void init(){
        CommandScheduler.getInstance().reset();
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad1);

        //CHASSIS CONTROLLER BINDS
        //INTAKE OPERATION
        new Trigger(()->chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .or(new Trigger(()-> chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1))
                .whileActiveContinuous(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
                                chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
                        )
                ))
                .whenInactive(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.intakeSubsystem.intakeStop())
                ));
    }
    public void loop(){
    CommandScheduler.getInstance().run();
    }
}
