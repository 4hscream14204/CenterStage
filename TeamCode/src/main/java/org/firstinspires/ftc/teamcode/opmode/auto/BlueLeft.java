package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;


@Autonomous(name = "BlueLeft")
public class BlueLeft extends OpMode {

    public RobotBase robotBase;
    private TrajectorySequence BlueLeftRightInner;
    private TrajectorySequence BlueLeftCenterInner;
    private TrajectorySequence BlueLeftCenterInner2;
    private TrajectorySequence BlueLeftLeftInner;

    private TrajectorySequence OuterPark;

    private TrajectorySequence InnerPark;

    private TrajectorySequence parkLocation;

    public Pose2d startPose;

    public GamepadEx autoChassisController;

    @Override
    public void init() {
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.parkSide = RobotBase.ParkSide.INNER;
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(15.00, 63.00, Math.toRadians(270.00));
        BlueLeftRightInner = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(6.00, 36.00), Math.toRadians(225.00))
                .lineToLinearHeading(new Pose2d(17.00, 46.00, Math.toRadians(270.00)))
                .addDisplacementMarker(() -> {robotBase.grabber.dropPosition();})
                .splineToLinearHeading(new Pose2d(40.00, 31.70, Math.toRadians(180.00)), Math.toRadians(360.00))
                .lineToLinearHeading(new Pose2d(52.00, 31.70, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.grabber.drop())
                .lineTo(new Vector2d(51.00, 31.70))
                .lineTo(new Vector2d(52.00,31.70))
                .waitSeconds(0.25)
                .lineToConstantHeading(new Vector2d(50.00, 31.70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(45.00, 36.70))
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})
               // .splineTo(new Vector2d(58.00, 61.00), Math.toRadians(360.00))
                .build();

        BlueLeftCenterInner = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineToSplineHeading(new Pose2d(12.00, 34.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(12.00, 39.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .addDisplacementMarker(() -> {robotBase.grabber.dropPosition();})
                .lineToSplineHeading(new Pose2d(35.00, 39.00, Math.toRadians(270.00)))
                //.lineToLinearHeading(new Pose2d(50.50, -37.50, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToSplineHeading(new Pose2d(52.0, 39.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.grabber.drop())
                .lineTo(new Vector2d(51.00, 39.00))
                .lineTo(new Vector2d(52.00,39.00))
                .waitSeconds(0.25)
                //.addTemporalMarker(() -> {robotBase.grabber.downPosition();})
                //.waitSeconds(1)
                .lineToLinearHeading(new Pose2d(45.00, 36.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                // .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build();

        BlueLeftCenterInner2 = robotBase.mecanumDrive.trajectorySequenceBuilder(BlueLeftCenterInner.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {robotBase.grabber.downPosition();})
                //.waitSeconds(1)
                //.lineToLinearHeading(new Pose2d(45.00, 36.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
               // .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build();

        BlueLeftLeftInner = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineToSplineHeading(new Pose2d(20.00, 37.00, Math.toRadians(300.00)), Math.toRadians(300.00))
                .lineToSplineHeading(new Pose2d(20.00, 43.00, Math.toRadians(270.00)))
                .addTemporalMarker(() -> robotBase.grabber.dropPosition())
                .lineToSplineHeading(new Pose2d(43.00, 43.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(52.00, 43.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.grabber.drop())
                .lineTo(new Vector2d(51.00, 43.00))
                .lineTo(new Vector2d(52.00,43.00))
                .waitSeconds(0.25)
                //.waitSeconds(1)
                //.addTemporalMarker(() -> robotBase.Grabber.DownPosition())
                .lineToSplineHeading(new Pose2d(45.0, 36.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.grabber.downPosition())
                //.splineTo(new Vector2d(60.00, 58.00), Math.toRadians(360.00))
                .build();

        OuterPark = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(45, 36, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(55.00, 61.00), Math.toRadians(0.00))
                .build();

        InnerPark = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(45, 36, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(55.00, 12.00), Math.toRadians(0.00))
                .build();

        robotBase.mecanumDrive.setPoseEstimate(startPose);
        parkLocation = InnerPark;
    }

    @Override
    public void init_loop() {
        autoChassisController.readButtons();
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.Y))) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }
            robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
            telemetry.addData("InitLoop", "true");
            telemetry.addData("Detection", (robotBase.propPosition));
            telemetry.addData("Park Side", (robotBase.parkSide));
            telemetry.update();

        }
        @Override
        public void start () {
            if (robotBase.propPosition == robotBase.propPosition.MIDDLE) {
                robotBase.mecanumDrive.followTrajectorySequence(BlueLeftCenterInner);
               // robotBase.grabber.drop();
               // robotBase.mecanumDrive.followTrajectorySequence(BlueLeftCenterInner2);
            } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
                robotBase.mecanumDrive.followTrajectorySequence(BlueLeftRightInner);
            } else {
                robotBase.mecanumDrive.followTrajectorySequence(BlueLeftLeftInner);
            }
            robotBase.mecanumDrive.followTrajectorySequence(parkLocation);

        }
        @Override
        public void loop () {

        }
        @Override
        public void stop () {
            Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double dblCurrentHeading = angles.firstAngle;
            DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        }
    }

