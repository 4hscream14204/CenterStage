package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;


@Autonomous(name = "BlueLeft")
public class BlueLeft extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence BlueLeftRightInner;
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
                .lineToLinearHeading(new Pose2d(47.00, 31.70, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToConstantHeading(new Vector2d(50.00, 31.70))
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(45.00, 31.70))
                .addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                .splineTo(new Vector2d(58.00, 61.00), Math.toRadians(360.00))
                .build();


        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.MecanumDrive.followTrajectorySequence(BlueLeftRightInner);
    }
    @Override
    public void loop(){

    }
}
