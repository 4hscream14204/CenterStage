package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.net.PortUnreachableException;

@Autonomous(name = "RedLeft")
public class RedLeft extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence RedLeftCenterInner;
    public Pose2d startPose;

    @Override
    public void init(){
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        RedLeftCenterInner = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.00, -32.28), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(-36.04, -41.52, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-28.50, -60.00), Math.toRadians(-1.41))
                .splineTo(new Vector2d(0.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(51.00, -36.50), Math.toRadians(0.00))
                .lineToSplineHeading(new Pose2d(51.00, -60.00, Math.toRadians(0.00)))
                .splineTo(new Vector2d(62.00, -60.00), Math.toRadians(1.33))
                .build();
        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.MecanumDrive.followTrajectorySequence(RedLeftCenterInner);
    }
    @Override
    public void loop(){

    }
}
