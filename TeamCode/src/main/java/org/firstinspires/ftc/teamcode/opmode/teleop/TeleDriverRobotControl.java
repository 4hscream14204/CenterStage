package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.AirplaneLaunchAndLowerCommand;
import org.firstinspires.ftc.teamcode.commands.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.DropOffPositionCommand;
import org.firstinspires.ftc.teamcode.commands.GrabAndWristEscapeCommandGrp;
import org.firstinspires.ftc.teamcode.commands.RaiseArmAndLauncherCommand;
import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private TriggerReader leftTriggerChassisReader;
    private TriggerReader rightTriggerChassisReader;
    private TriggerReader leftTriggerArmReader;
    private TriggerReader rightTriggerArmReader;
    private PIDFController headingControl;
    private double dblTargetHeading = 0;
    private double dblHeadingDeviation = 0;
    private double dblHeadingOutput = 0;
    private double dblCurrentTime = 0;
    private double dblDelayTime = 200;
    private double dblLastStickTime = 0;

    private double dblBackdropHeadingAngle = Math.toRadians(90);
    private double dblWingHeadingAngle = Math.toRadians(135);
    private double dblAirplaneLaunchingAngle = Math.toRadians(180);

    private ElapsedTime timer;

    public void init() {
        CommandScheduler.getInstance().reset();
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);

        robotBase.navxMicro.initialize();

        dblTargetHeading = DataStorageSubsystem.dblIMUFinalHeading;
        headingControl = new PIDFController(SampleMecanumDrive.HEADING_PID);
        headingControl.setTargetPosition(0);

        timer = new ElapsedTime();

        TriggerReader leftTriggerChassisReader = new TriggerReader(
                chassisController, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTriggerChassisReader = new TriggerReader(
                chassisController, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        TriggerReader leftTriggerArmReader = new TriggerReader(
                armController, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTriggerArmReader = new TriggerReader(
                armController, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        if(DataStorageSubsystem.alliance == RobotBase.Alliance.RED) {
            dblBackdropHeadingAngle = Math.toRadians(270);
            dblWingHeadingAngle = Math.toRadians(225);
            dblAirplaneLaunchingAngle = Math.toRadians(0);
        } else {
            dblBackdropHeadingAngle = Math.toRadians(90);
            dblWingHeadingAngle = Math.toRadians(135);
            dblAirplaneLaunchingAngle = Math.toRadians(180);
            DataStorageSubsystem.alliance = RobotBase.Alliance.BLUE;
        }

        //CHASSIS CONTROLLER BINDS
        //HANGING MECHANISM OPERATION
        chassisController.getGamepadButton(GamepadKeys.Button.Y)
                .and(new GamepadButton(chassisController, GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggle()));

        chassisController.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(()-> CommandScheduler.getInstance().schedule(new ConditionalCommand(
                                new InstantCommand(),
                                new InstantCommand(()->robotBase.hangingMechanismSubsystem.hangingToggleCheck()),
                                        ()->chassisController.getButton(GamepadKeys.Button.DPAD_UP)
                        )));

//new InstantCommand(() -> robotBase.hangingMechanismSubsystem.hangingToggleCheck())

        //ROBOT SLASH FIELD CENTRIC SWAP


        //IMU RESET
        chassisController.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    DataStorageSubsystem.dblIMUFinalHeading = 0;
                    robotBase.navxMicro.initialize();
                }));

        //CHASSIS BACKDROP POSITION
        chassisController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> dblTargetHeading = dblBackdropHeadingAngle)));

        //CHASSIS WING POSITION
        chassisController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> dblTargetHeading = dblWingHeadingAngle)
                ));

        //CHASSIS AIRPLANE LAUNCHER POSITION
        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(()->CommandScheduler.getInstance().schedule(
                                new InstantCommand(()-> dblTargetHeading = dblAirplaneLaunchingAngle)
                        ));

        //TESTING LIGHTS CODE
        new Trigger(()-> robotBase.leftTouchSensorSubsystem.pixelInIntake() == true)
                .whenActive(()->CommandScheduler.getInstance().schedule(
                        /*
                        new ParallelCommandGroup(
                            new GrabAndWristEscapeCommandGrp(
                            robotBase.leftWristSubsystem, robotBase.leftClawSubsystem),
                         */
                                new InstantCommand(()-> robotBase.leftLightsSubsystem.lightOn())
                        //) REMOVE COMMENT BRACKETS ONCE THIS IS CONFIRMED TO WORK
                ));

        new Trigger(()-> robotBase.rightTouchSensorSubsystem.pixelInIntake() == true)
                .whenActive(()->CommandScheduler.getInstance().schedule(
                        /*
                        new ParallelCommandGroup(
                            new GrabAndWristEscapeCommandGrp(
                            robotBase.rightWristSubsystem, robotBase.rightClawSubsystem),
                         */
                        new InstantCommand(()-> robotBase.rightLightsSubsystem.lightOn())
                        //) REMOVE COMMENT BRACKETS ONCE THIS IS CONFIRMED TO WORK
                ));

        /*
        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(()->CommandScheduler.getInstance().schedule(
                                new InstantCommand(()-> robotBase.rightLightsSubsystem.lightOn())
                        ));

        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.leftLightsSubsystem.lightOn())
                ));

        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.rightLightsSubsystem.lightOff())
                ));

        chassisController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenReleased(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> robotBase.leftLightsSubsystem.lightOff())
                ));
        */

        //INTAKE OPERATION
        new Trigger(()-> chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .or(new Trigger(()-> chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1))
                .whileActiveContinuous(()->CommandScheduler.getInstance().schedule(new InstantCommand(()->
                        robotBase.intakeSubsystem.intake(chassisController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) -
                                chassisController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER
                                )))))
                .whenInactive(()->CommandScheduler.getInstance().schedule(new InstantCommand(()->
                        robotBase.intakeSubsystem.intakeStop()
                )));

        //ARM CONTROLLER BINDS
        //LEFT CLAW DROPOFF

        armController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new ClawOpenCommand(robotBase.armSubsystem, robotBase.leftClawSubsystem));

        //RIGHT CLAW DROPOFF
        armController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ClawOpenCommand(robotBase.armSubsystem, robotBase.rightClawSubsystem));

        /*
        armController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new InstantCommand(()-> robotBase.leftClawSubsystem.clawOpen()));

        armController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> robotBase.rightClawSubsystem.clawOpen()));
*/

        //DUEL CLAW DROPOFF
        armController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ParallelCommandGroup(new ClawOpenCommand(robotBase.armSubsystem,
                        robotBase.rightClawSubsystem),
                        new ClawOpenCommand(robotBase.armSubsystem,
                                robotBase.leftClawSubsystem)));

        //DUEL CLAW CLOSING
        armController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ParallelCommandGroup(
                        new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                            robotBase.leftClawSubsystem),
                         new GrabAndWristEscapeCommandGrp(robotBase.rightWristSubsystem,
                            robotBase.rightClawSubsystem)
                ));

        //SLIDE MOVEMENTS
        //LEFT SLIDE LOWEST


        //RIGHT SLIDE LOWEST


        //DUAL SLIDE LOWEST
        armController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem,
                                        robotBase.leftClawSubsystem),
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem,
                                        robotBase.rightClawSubsystem)
                                ),
                                new ParallelCommandGroup(
                                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOWEST),
                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.rightWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOWEST))
                        )
                ));

        //LEFT SLIDE LOW


        //RIGHT SLIDE LOW


        //DUAL SLIDE LOW
        armController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem,
                                        robotBase.leftClawSubsystem),
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem,
                                        robotBase.rightClawSubsystem)
                        ),
                                new ParallelCommandGroup(
                                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOW),
                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.rightWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOW))
                        )
                ));

        //LEFT SLIDE LOW MEDIUM


        //RIGHT SLIDE LOW MEDIUM


        //DUAL SLIDE LOW MEDIUM
        armController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem,
                                        robotBase.leftClawSubsystem),
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem,
                                        robotBase.rightClawSubsystem)
                        ),
                                new ParallelCommandGroup(
                                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOWMEDIUM),
                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.rightWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.LOWMEDIUM))
                        )
                ));

        //LEFT SLIDE MEDIUM


        //RIGHT SLIDE MEDIUM


        //DUEL SLIDE MEDIUM
        armController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem,
                                        robotBase.leftClawSubsystem),
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem,
                                        robotBase.rightClawSubsystem)
                        ),
                                new ParallelCommandGroup(
                                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.MEDIUM),
                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.rightWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.MEDIUM))
                        )
                ));

        //DUEL SLIDE MEDIUM HIGH
        armController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.leftWristSubsystem,
                                        robotBase.leftClawSubsystem),
                                new GrabAndWristEscapeCommandGrp(
                                        robotBase.rightWristSubsystem,
                                        robotBase.rightClawSubsystem)
                        ),
                        new ParallelCommandGroup(
                                        new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.MEDIUMHIGH),
                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.rightWristSubsystem,
                                                robotBase.intakeSubsystem,
                                                RobotBase.SlideHeight.MEDIUMHIGH))
                        )
                ));

        //AIRPLANE LAUNCHER OPERATION
        armController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .and(armController.getGamepadButton(GamepadKeys.Button.START))
                                .whenActive(()-> CommandScheduler.getInstance().schedule(new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new GrabAndWristEscapeCommandGrp(
                                                                robotBase.leftWristSubsystem,
                                                                robotBase.leftClawSubsystem),
                                                        new GrabAndWristEscapeCommandGrp(
                                                                robotBase.rightWristSubsystem,
                                                                robotBase.rightClawSubsystem)
                                                ),
                                                new ParallelCommandGroup(
                                                new DropOffPositionCommand(robotBase.leftSlideSubsystem,
                                                        robotBase.armSubsystem,
                                                        robotBase.leftWristSubsystem,
                                                        robotBase.intakeSubsystem,
                                                        RobotBase.SlideHeight.LOWEST),
                                                        new DropOffPositionCommand(robotBase.rightSlideSubsystem,
                                                                robotBase.armSubsystem,
                                                                robotBase.rightWristSubsystem,
                                                                robotBase.intakeSubsystem,
                                                                RobotBase.SlideHeight.LOWEST)
                                                ),
                                                new AirplaneLaunchAndLowerCommand(robotBase.airplaneLauncherSubsystem,
                                                robotBase.leftClawSubsystem,
                                                robotBase.rightClawSubsystem)
                                        ),
                                        new RaiseArmAndLauncherCommand(robotBase.airplaneLauncherSubsystem,
                                                robotBase.armSubsystem,
                                                robotBase.leftClawSubsystem,
                                                robotBase.rightClawSubsystem),
                                        ()->robotBase.airplaneLauncherSubsystem.elevatorIsRaised()
                                                )));
    }

    public void loop() {
        dblCurrentTime = timer.milliseconds();

        chassisController.readButtons();
        armController.readButtons();
        dblChassisControllerRightX = Math.abs(chassisController.getRightX()) * chassisController.getRightX();
        dblChassisControllerRightY = Math.abs(chassisController.getRightY()) * chassisController.getRightY();
        dblChassisControllerLeftX = Math.abs(chassisController.getLeftX()) * chassisController.getLeftX();
        //dblCurrentHeading = robotBase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        dblCurrentHeading = angles.firstAngle + DataStorageSubsystem.dblIMUFinalHeading;

        //UNIVERSAL GRABBING COMMAND
        CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase));

        if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
            Vector2d input = new Vector2d(
                    -dblChassisControllerRightY,
                    -dblChassisControllerRightX
            ).rotated(-dblCurrentHeading);

            if(Math.abs(chassisController.getLeftX()) > 0.05) {
                dblLastStickTime = dblCurrentTime;

                robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -dblChassisControllerLeftX
                        )
                );

            } else if((dblCurrentTime - dblLastStickTime) < dblDelayTime){

                dblTargetHeading = dblCurrentHeading;

                robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -dblChassisControllerLeftX
                        )
                );

            } else {

                dblHeadingDeviation = dblCurrentHeading - dblTargetHeading;
                dblHeadingDeviation = Angle.normDelta(dblHeadingDeviation);

                dblHeadingOutput = headingControl.update(dblHeadingDeviation) * DriveConstants.kV * DriveConstants.TRACK_WIDTH;

                robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                dblHeadingOutput
                        )
                );

            }


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
            robotBase.mecanumDriveSubsystem.chassisDirectionalSwap(hardwareMap);
        }

            robotBase.mecanumDriveSubsystem.update();

            telemetry.addData("IMU yaw angle", Math.toDegrees(angles.firstAngle));
            telemetry.addData("Chassis Control", robotBase.controlScheme);
            telemetry.addData("Arm Position", robotBase.armSubsystem.getArmPosition());
            telemetry.addData("Current Heading", Math.toDegrees(dblCurrentHeading));
            telemetry.addData("Target Heading", Math.toDegrees(dblTargetHeading));
            telemetry.addData("Alliance Side", DataStorageSubsystem.alliance);
            CommandScheduler.getInstance().run();
    }
    public void stop (){
        robotBase.leftLightsSubsystem.lightOff();
        robotBase.rightLightsSubsystem.lightOff();
    }
}