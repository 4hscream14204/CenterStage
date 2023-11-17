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


@Autonomous(name = "RedRight")
public class RedRight extends OpMode {

    public RobotBase robotBase;
    private TrajectorySequence RedRightLeftInner;
    private TrajectorySequence RedRightCenterInner;
    private TrajectorySequence RedRightCenterInner2;
    private TrajectorySequence RedRightRightInner;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        telemetry.update();
        startPose = new Pose2d(15.00, -63.00, Math.toRadians(90.00));
        RedRightLeftInner = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(6.00, -36.00), Math.toRadians(135.00))
                .lineToLinearHeading(new Pose2d(17.00, -46.00, Math.toRadians(90.00)))
                //.addDisplacementMarker(() -> {robotBase.grabber.dropPosition();})
                .splineToLinearHeading(new Pose2d(40.00, -31.70, Math.toRadians(180.00)), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(52.00, -31.70, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> robotBase.Grabber.Drop())
                //.addTemporalMarker(() -> {robotBase.grabber.drop();})
                .waitSeconds(1)
                //.addTemporalMarker(() -> robotBase.grabber.downPosition())
                .lineToLinearHeading(new Pose2d(50.00, -31.70, Math.toRadians(180.00)))
                //.waitSeconds(1)
                .lineToConstantHeading(new Vector2d(45.00, -31.70))
                //.addDisplacementMarker(() -> {robotBase.grabber.downPosition();})
                .splineTo(new Vector2d(58.00, -61.0), Math.toRadians(0.00))
                .build();

        RedRightCenterInner = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(12.00, -37.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                //.addDisplacementMarker(() -> {robotBase.grabber.dropPosition();})
                .lineToSplineHeading(new Pose2d(35.00, -38.00, Math.toRadians(180.00)))
                //.lineToLinearHeading(new Pose2d(50.50, -37.50, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToSplineHeading(new Pose2d(51.50, -38.00, Math.toRadians(180.00)))
                //.waitSeconds(3)
                //.lineToLinearHeading(new Pose2d(40.00, -37.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

        RedRightCenterInner2 = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(RedRightCenterInner.end())
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50.00, -37.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.grabber.downPosition();})
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

        RedRightRightInner = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(20.00, -37.00, Math.toRadians(60.00)), Math.toRadians(60.00))
                .lineToSplineHeading(new Pose2d(20.00, -43.00, Math.toRadians(90.00)))
                //.addTemporalMarker(() -> robotBase.grabber.dropPosition())
                .lineToSplineHeading(new Pose2d(43.00, -43.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(52.00, -43.00, Math.toRadians(180.00)))
                //.addTemporalMarker(() -> robotBase.grabber.drop())
                .waitSeconds(1)
                //.addTemporalMarker(() -> robotBase.Grabber.DownPosition())
                .lineToSplineHeading(new Pose2d(50.00, -43.00, Math.toRadians(180.00)))
                //.addTemporalMarker(() -> robotBase.grabber.downPosition())
                .splineTo(new Vector2d(59.00, -58.00), Math.toRadians(0.00))
                //.splineToSplineHeading(new Pose2d(59.00, -58.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
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
        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RedRightCenterInner);
            //robotBase.grabber.drop();
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RedRightCenterInner2);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RedRightRightInner);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RedRightLeftInner);
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
