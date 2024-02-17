package org.firstinspires.ftc.teamcode.opmode.auto;

import android.provider.ContactsContract;

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
    private TrajectorySequence RightSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence LeftSpike;

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
        robotBase.leftClawSubsystem.clawClose();
        robotBase.leftWristSubsystem.wristEscape();
        startPose = new Pose2d(15.00, 63.00, Math.toRadians(270.00));
        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.50, 63.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(3.00, 38.00, Math.toRadians(225.00)), Math.toRadians(225.00))
                .lineTo(new Vector2d(16.00, 52.00))
                .splineToLinearHeading(new Pose2d(32.95, 35.16, Math.toRadians(0.00)), Math.toRadians(270.00))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(50.00, 28.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(1.5)
                .addTemporalMarker(3, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(3.5, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .waitSeconds(0.5)
                .addTemporalMarker(7, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .lineTo(new Vector2d(40, 28))
                .waitSeconds(1)
                .addTemporalMarker(8.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .waitSeconds(0.5)
                .addTemporalMarker(9, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .waitSeconds(1)
               // .lineToConstantHeading(new Vector2d(45.00, 36.70))
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.50, 63.00, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(10.00, 35.00), Math.toRadians(270.00))
                .lineTo(new Vector2d(16.00, 52.00))
                .splineToLinearHeading(new Pose2d(33, 37, Math.toRadians(0.00)), Math.toRadians(270.00))
                .splineTo(new Vector2d(50.00, 36.00), Math.toRadians(00))
                .waitSeconds(1.5)
                .addTemporalMarker(3, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(3.5, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .waitSeconds(0.5)
                .addTemporalMarker(6, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .lineTo(new Vector2d(40, 28))
                .waitSeconds(1)
                .addTemporalMarker(8.5, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .waitSeconds(0.5)
                .addTemporalMarker(9, () -> { robotBase.armSubsystem.armGrabbingPosition();})
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
                // .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build();



        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(17.50, 63.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(15, 35.00), Math.toRadians(315.00))
                .lineTo(new Vector2d(15.00, 43.69))
                .splineToSplineHeading(new Pose2d(40.00, 43.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(1.5)
                .addTemporalMarker(3, () -> { robotBase.armSubsystem.armDropOffLowestPos();})
                .addTemporalMarker(3.5, () -> { robotBase.leftWristSubsystem.wristDropOffLowest();})
                .lineTo(new Vector2d(50,43))
                .waitSeconds(1)
                .lineTo(new Vector2d(40, 28))
                .addTemporalMarker(7, () -> { robotBase.leftClawSubsystem.clawOpen();})
                .waitSeconds(0.5)
                .addTemporalMarker(8, () -> { robotBase.leftWristSubsystem.wristPickup();})
                .waitSeconds(0.5)
                .addTemporalMarker(8.5, () -> { robotBase.armSubsystem.armGrabbingPosition();})
                .build();

        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .lineTo(new Vector2d(45.00, 62.00))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, 36.00, Math.toRadians(0)))
                .lineTo(new Vector2d(45.00, 12.00))
                //.lineTo(new Vector2d(57.00, 12.00))
                .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
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
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(MiddleSpike);
            // robotBase.grabber.drop();
            // robotBase.mecanumDrive.followTrajectorySequence(BlueLeftCenterInner2);
        } else if (robotBase.propPosition == RobotBase.PropPosition.LEFT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(LeftSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RightSpike);
        }
        robotBase.mecanumDriveSubsystem.followTrajectorySequence(parkLocation);

    }
    @Override
    public void loop () {

    }
    @Override
    public void stop () {
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.BLUE;
    }
}
