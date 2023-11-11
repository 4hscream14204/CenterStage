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
    private TrajectorySequence RedLeftCenterOuter;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        RedLeftCenterOuter = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.00, -35.00), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(-29.39, -35.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(51.00, -35.0), Math.toRadians(90.00))
                //.lineToConstantHeading(new Vector2d(51.00, -12.00))
                //.splineTo(new Vector2d(60.00, -12.00), Math.toRadians(-0.80))
                .build();
        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.MecanumDrive.followTrajectorySequence(RedLeftCenterOuter);
    }
    @Override
    public void loop(){

    }
}
