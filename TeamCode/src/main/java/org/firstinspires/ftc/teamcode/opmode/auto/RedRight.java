package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;


@Autonomous(name = "RedRight")
public class RedRight extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence RedRightLeftInner;
    private TrajectorySequence RedRightCenterInner;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        startPose = new Pose2d(15.00, -63.00, Math.toRadians(90.00));
        RedRightLeftInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(6.00, -36.00), Math.toRadians(135.00))
                .lineToLinearHeading(new Pose2d(17.00, -46.00, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.DropPosition();})
                .splineToLinearHeading(new Pose2d(40.00, -31.70, Math.toRadians(180.00)), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(47.00, -31.70, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToLinearHeading(new Pose2d(49.00, -31.70, Math.toRadians(180.00)))
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(45.00, -31.70))
                .addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .splineTo(new Vector2d(58.00, -60.0), Math.toRadians(0.00))
                .build();

        RedRightCenterInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(12.00, -37.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {robotBase.Grabber.DropPosition();})
                .lineToSplineHeading(new Pose2d(35.00, -37.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(46.00, -37.00, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToSplineHeading(new Pose2d(48.50, -37.00, Math.toRadians(180.00)))
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(40.00, -37.00, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        //robotBase.MecanumDrive.followTrajectorySequence(RedRightLeftInner);
        robotBase.MecanumDrive.followTrajectorySequence(RedRightCenterInner);
    }
    @Override
    public void loop(){

    }
}
