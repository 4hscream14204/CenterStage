package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(15.00, 63.00, Math.toRadians(270.00));
        BlueLeftRightInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(6.00, 36.00), Math.toRadians(225.00))
                .lineToLinearHeading(new Pose2d(17.00, 46.00, Math.toRadians(270.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.DropPosition();})
                .splineToLinearHeading(new Pose2d(40.00, 31.70, Math.toRadians(180.00)), Math.toRadians(360.00))
                .lineToLinearHeading(new Pose2d(52.00, 31.70, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToConstantHeading(new Vector2d(50.00, 31.70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(45.00, 31.70))
                .addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                .splineTo(new Vector2d(58.00, 61.00), Math.toRadians(360.00))
                .build();

        BlueLeftCenterInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineToSplineHeading(new Pose2d(12.00, 34.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(12.00, 39.00, Math.toRadians(270.00)), Math.toRadians(270.00))
                .addDisplacementMarker(() -> {robotBase.Grabber.DropPosition();})
                .lineToSplineHeading(new Pose2d(35.00, 39.00, Math.toRadians(270.00)))
                //.lineToLinearHeading(new Pose2d(50.50, -37.50, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToSplineHeading(new Pose2d(52.0, 39.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.Grabber.Drop())
                //.waitSeconds(3)
                //.lineToLinearHeading(new Pose2d(40.00, -37.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

        BlueLeftCenterInner2 = robotBase.MecanumDrive.trajectorySequenceBuilder(BlueLeftCenterInner.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.waitSeconds(1)
                .lineToLinearHeading(new Pose2d(40.00, 37.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build();

        BlueLeftLeftInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, 63.00, Math.toRadians(270.00)))
                .splineToSplineHeading(new Pose2d(20.00, 37.00, Math.toRadians(300.00)), Math.toRadians(300.00))
                .lineToSplineHeading(new Pose2d(20.00, 43.00, Math.toRadians(270.00)))
                .addTemporalMarker(() -> robotBase.Grabber.DropPosition())
                .lineToSplineHeading(new Pose2d(43.00, 43.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(52.00, 43.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.Grabber.Drop())
                .waitSeconds(1)
                //.addTemporalMarker(() -> robotBase.Grabber.DownPosition())
                .lineToSplineHeading(new Pose2d(46.50, 43.00, Math.toRadians(180.00)))
                .addTemporalMarker(() -> robotBase.Grabber.DownPosition())
                .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(360.00))
                .build();

        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){
        robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
        telemetry.addData("InitLoop","true");
        telemetry.addData("Detection",(robotBase.propPosition));
        telemetry.update();
    }
    @Override
    public void start(){
        if (robotBase.propPosition == robotBase.propPosition.MIDDLE) {
            robotBase.MecanumDrive.followTrajectorySequence(BlueLeftCenterInner);
            robotBase.Grabber.Drop();
            robotBase.MecanumDrive.followTrajectorySequence(BlueLeftCenterInner2);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.MecanumDrive.followTrajectorySequence(BlueLeftRightInner);
        } else {
            robotBase.MecanumDrive.followTrajectorySequence(BlueLeftLeftInner);
        }
    }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
    }
}
