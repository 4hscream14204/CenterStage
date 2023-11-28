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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;

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
    private TriggerReader leftTriggerArmReader;
    private TriggerReader rightTriggerArmReader;

    public void init() {
        CommandScheduler.getInstance().reset();
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);

        TriggerReader leftTriggerArmReader = new TriggerReader(
                armController, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTriggerArmReader = new TriggerReader(
                armController, GamepadKeys.Trigger.RIGHT_TRIGGER
        );


        //CHASSIS CONTROLLER BINDS
        //HANGING MECHANISM OPERATION
        switch (robotBase.hangingState) {
            case DOWN:
            chassisController.getGamepadButton(GamepadKeys.Button.Y)
                    .and(new GamepadButton(chassisController, GamepadKeys.Button.DPAD_UP))
                    .whenActive(new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggle()));
            break;
            case RAISED:
            case LOWERED:
                chassisController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggle()));
                break;
        }

        //ROBOT SLASH FIELD CENTRIC SWAP

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
        robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));


        //ARM CONTROLLER BINDS

        if (syncSlidesMode == RobotBase.SyncSlidesMode.OFF) {
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
           if (rightTriggerArmReader.wasJustPressed()) {
                robotBase.slideSubsystem.rightSlidePositionLower();
            }
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
             if (leftTriggerArmReader.wasJustPressed()) {
                 robotBase.slideSubsystem.leftSlidePositionLower();
             }
            if (armController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                robotBase.clawSubsystem.leftOpen();
            }
        } else {
            if (armController.wasJustPressed(GamepadKeys.Button.A) || armController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                robotBase.slideSubsystem.dualLowToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.B) || armController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robotBase.slideSubsystem.dualMediumToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.Y) && !armController.getButton(GamepadKeys.Button.START) || armController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                robotBase.slideSubsystem.dualHighToggle();
            }
            if (armController.wasJustPressed(GamepadKeys.Button.X) || armController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                robotBase.slideSubsystem.dualSlidePositionRaise();
            }
            if (rightTriggerArmReader.wasJustPressed() || leftTriggerArmReader.wasJustPressed()) {
                robotBase.slideSubsystem.dualSlidePositionLower();
            }
        }
        //OTHER ARM BINDS
        if (armController.wasJustPressed(GamepadKeys.Button.BACK)) {
            robotBase.slideSubsystem.syncSlidesToggle();
        }
        if (armController.wasJustPressed(GamepadKeys.Button.Y) && armController.getButton(GamepadKeys.Button.START)) {
            robotBase.airplaneLauncherSubsystem.raiseAndLaunch();
        }
            robotBase.mecanumDriveSubsystem.update();

            telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
            telemetry.addData("Chassis Control", robotBase.controlScheme);
        CommandScheduler.getInstance().run();
    }
}
