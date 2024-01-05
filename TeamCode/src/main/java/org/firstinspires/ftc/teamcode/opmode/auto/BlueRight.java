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

@Autonomous(name = "BlueRight")
public class BlueRight extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence BlueRightPark;
    public Pose2d startPose;
    public TrajectorySequence leftSpike;
    public TrajectorySequence middleSpike;
    public TrajectorySequence rightSpike;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));
        BlueRightPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, 63.30, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-38.35, 12.00, Math.toRadians(0.00)))
                .waitSeconds(20)
                .lineTo(new Vector2d(6.00, 12.00))
                .lineTo(new Vector2d(50.00, 12.00))
                .build();

        leftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-36.00, 42.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                .lineToLinearHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-52, 600), Math.toRadians(0))
                .lineTo(new Vector2d(25, 59))
                .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(0))
                        .build();

        middleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)), Math.toRadians(315.00))
                .lineToLinearHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-52, 600), Math.toRadians(0))
                .lineTo(new Vector2d(25, 59))
                .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(0))
                        .build();

        rightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, 63.3, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-37.00, 35.00, Math.toRadians(220.00)), Math.toRadians(261.00))
                .lineToLinearHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-52, 600), Math.toRadians(0))
                .lineTo(new Vector2d(25, 59))
                .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(0))
                        .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.mecanumDriveSubsystem.followTrajectorySequence(BlueRightPark);
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
