package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.DropOffPositionCommand;
import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
        chassisController.getGamepadButton(GamepadKeys.Button.Y)
                .and(new GamepadButton(chassisController, GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggle()));

        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggleCheck()));

        //ROBOT SLASH FIELD CENTRIC SWAP


        //IMU RESET
        chassisController.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    DataStorageSubsystem.dblIMUFinalHeading = 0;
                    robotBase.navxMicro.initialize();
                }));

        //DUEL CLAW CLOSING
        chassisController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    robotBase.leftClawSubsystem.clawClose();
                    robotBase.rightClawSubsystem.clawClose();
                }));

        //CHASSIS BACKDROP POSITION
        chassisController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(/* chassis backdrop position */));

        //CHASSIS WING POSITION
        chassisController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(/* chassis wing position */));

        //ARM CONTROLLER BINDS
        //LEFT CLAW DROPOFF
        armController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new ClawOpenCommand(robotBase.armSubsystem, robotBase.leftClawSubsystem));

        //RIGHT CLAW DROPOFF
        armController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ClawOpenCommand(robotBase.armSubsystem, robotBase.rightClawSubsystem));

        //SLIDE MOVEMENTS
        //LEFT SLIDE LOW


        //RIGHT SLIDE LOW


        //DUAL SLIDE LOW
        armController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.leftWristSubsystem,
                                robotBase.leftClawSubsystem,
                                robotBase.rightClawSubsystem,
                                RobotBase.SlideHeight.LOW),
                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.rightWristSubsystem,
                                robotBase.leftClawSubsystem,
                                robotBase.rightClawSubsystem,
                                RobotBase.SlideHeight.LOW)));

        //LEFT SLIDE MEDIUM


        //RIGHT SLIDE MEDIUM


        //DUAL SLIDE MEDIUM
        armController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.leftWristSubsystem,
                                robotBase.leftClawSubsystem,
                                robotBase.rightClawSubsystem,
                                RobotBase.SlideHeight.MEDIUM),
                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                robotBase.armSubsystem,
                                robotBase.rightWristSubsystem,
                                robotBase.leftClawSubsystem,
                                robotBase.rightClawSubsystem,
                                RobotBase.SlideHeight.MEDIUM)));

        //LEFT SLIDE HIGH


        //RIGHT SLIDE HIGH


        //DUAL SLIDE HIGH


        //LEFT BACKDROP POSITION RAISE


        //LEFT BACKDROP POSITION LOWER


        //RIGHT BACKDROP POSITION RAISE


        //RIGHT BACKDROP POSITION LOWER


        //DUAL BACKDROP POSITION RAISE


        //DUAL BACKDROP POSITION LOWER


        //AIRPLANE LAUNCHER OPERATION


        //CommandScheduler.getInstance().setDefaultCommand(UniversalGrabbingPosCommand());

        armController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()->{
                    telemetry.addLine("X Button Pressed");
                    robotBase.armSubsystem.armDropOffPos();
                }));
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

        //INTAKE OPERATION
        if (chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
                    chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.2 || chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.2) {
            robotBase.intakeSubsystem.intakeStop();
        }
            robotBase.mecanumDriveSubsystem.update();

            telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
            telemetry.addData("Chassis Control", robotBase.controlScheme);
        CommandScheduler.getInstance().run();
    }
}