package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name="DriverRobotControl")
public class TeleDriverRobotControl extends OpMode {


    public RobotBase robotBase;
    private RobotBase.SyncSlidesMode syncSlidesMode;
    private RobotBase.HangingState hangingState;
    private GamepadEx chassisController;
    private GamepadEx armController;
    private double dblCurrentHeading = 0;
    private double dblChassisControllerRightX = 0;
    private double dblChassisControllerRightY = 0;
    private double dblChassisControllerLeftX = 0;
    private String strLastButtonPressed = "";


    public void init() {
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);

        TriggerReader triggerReader = new TriggerReader(
                armController, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader triggerReader1 = new TriggerReader(
                armController, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
    }

    public void loop() {
        chassisController.readButtons();
        armController.readButtons();
        dblChassisControllerRightX = Math.abs(chassisController.getRightX()) * chassisController.getRightX();
        dblChassisControllerRightY = Math.abs(chassisController.getRightY()) * chassisController.getRightY();
        dblChassisControllerLeftX = Math.abs(chassisController.getLeftX()) * chassisController.getLeftX();
        //dblCurrentHeading = robotBase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        dblCurrentHeading = angles.firstAngle + DataStorageSubsystem.dblIMUFinalHeading;


        if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
            Vector2d input = new Vector2d(
                    -dblChassisControllerRightY,
                    -dblChassisControllerRightX
            ).rotated(-dblCurrentHeading);

            robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -dblChassisControllerLeftX
                    )
            );

        } else {

            robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                    new Pose2d(
                            -dblChassisControllerRightY,
                            -dblChassisControllerRightX,
                            -dblChassisControllerLeftX
                    )
            );


        }

        //CHASSIS CONTROLLER BINDS
        if (chassisController.wasJustPressed(GamepadKeys.Button.B)) {
            robotBase.hangingMechanismSubsystem.lower();
        }
        if (hangingState == RobotBase.HangingState.DOWN) {
            if (chassisController.wasJustPressed(GamepadKeys.Button.DPAD_UP) && chassisController.wasJustPressed(GamepadKeys.Button.Y)) {
                robotBase.hangingMechanismSubsystem.raisePosition();
            }
        } else {
            if (chassisController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robotBase.hangingMechanismSubsystem.raise();
            }
            if (chassisController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robotBase.hangingMechanismSubsystem.lower();
            }
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.START)) {
            if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
                robotBase.controlScheme = RobotBase.ChassisControlType.ROBOTCENTRIC;
            } else {
                robotBase.controlScheme = RobotBase.ChassisControlType.FIELDCENTRIC;
            }
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.BACK)) {
            DataStorageSubsystem.dblIMUFinalHeading = 0;
            robotBase.navxMicro.initialize();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.X)) {
            robotBase.clawSubsystem.leftClosed();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.B)) {
            robotBase.clawSubsystem.rightClosed();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.A)) {
            robotBase.clawSubsystem.leftClosed();
            robotBase.clawSubsystem.rightClosed();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            //chassis backdrop position
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            //chassis wing position
        }
        robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        robotBase.intakeSubsystem.outake(chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        /*
        if() {
            robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        }
        if() {
            robotBase.intakeSubsystem.outake(chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        }
        */

        //ARM CONTROLLER BINDS

        if (syncSlidesMode == RobotBase.SyncSlidesMode.Off) {
            //OPERATING THE RIGHT SLIDE AND CLAW
            if (armController.wasJustPressed(GamepadKeys.Button.A)) {
                robotBase.slideSubsystem.rightLowToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.B)) {
                robotBase.slideSubsystem.rightMediumToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.Y) && !armController.getButton(GamepadKeys.Button.START)) {
                robotBase.slideSubsystem.rightHighToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.X)) {
                robotBase.slideSubsystem.rightSlidePositionRaise();
            }
           // if (armController.wasJustPressed(GamepadKeys.Trigger.RIGHT_TRIGGER)) {
           //     robotBase.slideSubsystem.rightSlidePositionLower();
           // }
            if (armController.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robotBase.clawSubsystem.rightOpen();
            }
            //OPERATING THE LEFT SLIDE AND CLAW
            if (armController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robotBase.slideSubsystem.leftLowToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robotBase.slideSubsystem.leftMediumToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robotBase.slideSubsystem.leftHighToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                robotBase.slideSubsystem.leftSlidePositionRaise();
            }
            // if (armController.wasJustPressed(GamepadKeys.Trigger.LEFT_TRIGGER)) {
            //     robotBase.slideSubsystem.leftSlidePositionLower();
            // }
            if (armController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robotBase.clawSubsystem.leftOpen();
            }
        } else {
            if (armController.wasJustPressed(GamepadKeys.Button.A) || armController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robotBase.slideSubsystem.rightLowToggle();
                robotBase.slideSubsystem.leftLowToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.B)) {
                robotBase.slideSubsystem.rightMediumToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.Y) && !armController.getButton(GamepadKeys.Button.START)) {
                robotBase.slideSubsystem.rightHighToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.X)) {
                robotBase.slideSubsystem.rightSlidePositionRaise();
            }
            // if (armController.wasJustPressed(GamepadKeys.Trigger.RIGHT_TRIGGER)) {
            //     robotBase.slideSubsystem.rightSlidePositionLower();
            // }
        }
            robotBase.mecanumDriveSubsystem.update();

            telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
            telemetry.addData("Chassis Control", robotBase.controlScheme);
            telemetry.addData("Button Pressed", strLastButtonPressed);
    }
}
