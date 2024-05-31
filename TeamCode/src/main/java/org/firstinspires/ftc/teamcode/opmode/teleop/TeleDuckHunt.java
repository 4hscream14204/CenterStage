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

    public TimerSubsystem timerSubsystem;



    public void init(){
        CommandScheduler.getInstance().reset();
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);

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

            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            /*
            if(timerSubsystem.timerIsPassed(DataStorageSubsystem.INTTIMERLENGTH, timer)) {
                stop();
            }
            */

            new Trigger(()->timerSubsystem.timerIsPassed(DataStorageSubsystem.INTTIMERLENGTH, runtime))
                    .whenActive(()-> CommandScheduler.getInstance().schedule(
                            new InstantCommand(()-> requestOpModeStop())
                    ));

            robotBase.leftFrontDrive.setPower(leftFrontPower);
            robotBase.rightFrontDrive.setPower(rightFrontPower);
            robotBase.leftBackDrive.setPower(leftBackPower);
            robotBase.rightBackDrive.setPower(rightBackPower);
    CommandScheduler.getInstance().run();
    }
}
