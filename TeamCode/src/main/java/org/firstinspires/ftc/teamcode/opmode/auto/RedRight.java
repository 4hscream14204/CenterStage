package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.DropOffPositionLowCommandGrp;
import org.firstinspires.ftc.teamcode.commands.UniversalGrabbingPosCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LogitechCameraSubsystemBetter;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name = "RedRight")
public class RedRight extends OpMode {

    public RobotBase robotBase;
    private TrajectorySequence RightSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence LeftSpike;

    private TrajectorySequence OuterPark;
    private TrajectorySequence MiddlePark;
    private TrajectorySequence InnerPark;
    //  private TrajectorySequence InnerStackPickup;
    //   private TrajectorySequence OuterStackPickup;
    private TrajectorySequence parkLocation;

    /*public TrajectorySequence tsStackPickup; */
    public Pose2d startPose;

    private enum CurrentRouteState {
        TRAJECTORY_1,
        PARKING,
        STACK
    }

    public GamepadEx autoChassisController;
    private RedRight.CurrentRouteState currentRouteState;

    private LogitechCameraSubsystemBetter visionProcesser;
    private VisionPortal visionPortal;
    private double timer = 0;
    private TrajectorySequence timewait;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.OUTER;
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        // robotBase.stackState = RobotBase.StackState.NONE;
        visionProcesser = new LogitechCameraSubsystemBetter(RobotBase.StartPosition.RIGHT);
        robotBase.leftClawSubsystem.clawClose();
        robotBase.leftWristSubsystem.wristEscape();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(visionProcesser)
                .setCameraResolution(new Size(864, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        startPose = new Pose2d(17.50, -63.00, Math.toRadians(90.00));
        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.5, -63.00, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(15.00, -34.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24,-49, Math.toRadians(0)), Math.toRadians(330))
              //  .lineTo(new Vector2d(24,-48))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45, -45), Math.toRadians(0.00))
                .addTemporalMarker( 2.5,() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(50.00,-45.00))
                .waitSeconds(.5)
                .addTemporalMarker( () -> {
                    robotBase.leftClawSubsystem.clawOpen();
                })
                .lineTo(new Vector2d(40, -36))
               /* .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    robotBase.leftWristSubsystem.wristPickup();
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    robotBase.armSubsystem.armGrabbingPosition();
                })
                .waitSeconds(.5) */
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.50, -63.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(10.00, -35.00), Math.toRadians(90.00))
                .setReversed(true)
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToLinearHeading(new Pose2d(45.00, -39.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(50.00, -39.00), Math.toRadians(180.00))
                .waitSeconds(0.2)
                .addTemporalMarker( () -> {
                    robotBase.leftClawSubsystem.clawOpen();
                })
                .lineTo(new Vector2d(40, -36))
                .waitSeconds(0.25)
               /* .addTemporalMarker( () -> {
                    robotBase.leftWristSubsystem.wristPickup();
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    robotBase.armSubsystem.armGrabbingPosition();
                })
                .waitSeconds(1)
                //.lineToSplineHeading(new Pose2d(35.00, 39.00, Math.toRadians(270.00)))
                //.lineToLinearHeading(new Pose2d(50.50, -37.50, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                //.lineToSplineHeading(new Pose2d(52.0, 39.00, Math.toRadians(180.00)))
                //.lineTo(new Vector2d(51.00, 39.00))
                //.lineTo(new Vector2d(52.00,39.00))
                //.waitSeconds(0.25)
                //.addTemporalMarker(() -> {robotBase.grabber.downPosition();})
                //.waitSeconds(1)
                //.lineToLinearHeading(new Pose2d(45.00, 36.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                // .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00)) */
                .build();


        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.50, -63.00, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(3.00, -36.00, Math.toRadians(135.00)), Math.toRadians(135.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(15,-40, Math.toRadians(0.00)),Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                        robotBase.armSubsystem,
                        robotBase.leftWristSubsystem,
                        robotBase.intakeSubsystem,
                        RobotBase.SlideHeight.LOWEST)))
                .splineToSplineHeading(new Pose2d(40.00, -41.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(50.00,-30.00))
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {robotBase.leftClawSubsystem.clawOpen();})
                .waitSeconds(0.5)
                .lineTo(new Vector2d(40.00, -36.00))
                .waitSeconds(0.2)
             /*   .addTemporalMarker( () -> {robotBase.leftWristSubsystem.wristPickup();})
                .waitSeconds(0.2)
                .addTemporalMarker( () -> {robotBase.armSubsystem.armGrabbingPosition();}) */
                .build();

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(0.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(43, -61), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(62, -64), Math.toRadians(0.00))
                .build();

       /* MiddlePark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(50, 36), Math.toRadians(0))
                .build(); */

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(43, -17), Math.toRadians(0.00))
                .build();
/*
        OuterStackPickup = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(25, 63), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-98, 63), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-108, 50, Math.toRadians(45)), Math.toRadians(180))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> robotBase.rakeSubsystem.rakePosition(0.8))
                ))
                .setReversed(false)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-98, 63), Math.toRadians(0.00))
                .addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> robotBase.rakeSubsystem.rakePosition(0))
                ))
                .splineToConstantHeading(new Vector2d(25, 61), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45, 36), Math.toRadians(270))
                .build();

 */

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
        parkLocation = OuterPark;
    }

    @Override
    public void init_loop() {
        autoChassisController.readButtons();

        // if (autoChassisController.wasJustPressed(GamepadKeys.Button.B)) {
        //   if (robotBase.stackState == RobotBase.StackState.OUTER) {
        // tsStackPickup = InnerStackPickup;
        //     robotBase.stackState = RobotBase.StackState.INNER;
        //  } else if (robotBase.stackState == RobotBase.StackState.INNER) {
        //  robotBase.stackState = RobotBase.StackState.NONE;
        //    } else {
        //   tsStackPickup = OuterStackPickup;
        //   robotBase.stackState = RobotBase.StackState.OUTER;
        //   }



        // switch (robotBase.stackState) {
        // case OUTER:
        //   tsStackPickup = InnerStackPickup;
        // robotBase.stackState = RobotBase.StackState.INNER;
        // case INNER:
        //   robotBase.stackState = RobotBase.StackState.NONE;
        // case NONE:
        // tsStackPickup = OuterStackPickup;
        //    robotBase.stackState = RobotBase.StackState.OUTER;
        //      }

        //   }

        if (autoChassisController.wasJustPressed(GamepadKeys.Button.Y)) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else if (robotBase.parkSide == RobotBase.ParkSide.OUTER) {
                robotBase.parkSide = RobotBase.ParkSide.MIDDLE;
                parkLocation = MiddlePark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.DPAD_UP))) {
            timer = timer + 1;
        }

        if (autoChassisController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            timer = timer - 1;
        }

        robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);

        robotBase.propPosition = visionProcesser.getLocation();

        telemetry.addData("InitLoop", "true");
        telemetry.addData("Detection", (robotBase.propPosition));
        telemetry.addData("Park Side", (robotBase.parkSide));
        //  telemetry.addData("Cycle State", (robotBase.stackState));
        telemetry.addData("TimerValue", (timer));
        telemetry.update();

    }
    @Override
    public void start () {
        if (timer > 0) {
            timewait = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(startPose)
                    .waitSeconds(timer)
                    .build();
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(timewait);
        }
        visionPortal.stopStreaming();
        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(MiddleSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.LEFT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(LeftSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(RightSpike);
        }

        currentRouteState = RedRight.CurrentRouteState.TRAJECTORY_1;
    }
    @Override
    public void loop () {
        switch (currentRouteState) {
            case TRAJECTORY_1:
                if (!robotBase.mecanumDriveSubsystem.isBusy()) {
                    currentRouteState = RedRight.CurrentRouteState.PARKING;
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new UniversalGrabbingPosCommand(robotBase),
                                    new WaitCommand(200),
                                    new InstantCommand(()->
                                            robotBase.mecanumDriveSubsystem.followTrajectorySequenceAsync(parkLocation))
                            )
                    );
                }
        }
        robotBase.mecanumDriveSubsystem.update();
        CommandScheduler.getInstance().run();
        telemetry.addData("TimerValue", (timer));
        telemetry.update();
    }

    @Override
    public void stop () {
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
    }
}
