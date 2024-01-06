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

@Autonomous(name = "RedLeft")
public class RedLeft extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence RedLeftPark;
    public Pose2d startPose;
    private TrajectorySequence leftSpike;
    private TrajectorySequence centerSpike;
    private TrajectorySequence rightSpike;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        RedLeftPark  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-38.35, -12.00, Math.toRadians(0.00)))
                .waitSeconds(20)
                .lineTo(new Vector2d(6.00, -12.00))
                .lineTo(new Vector2d(50, -12.00))
                .build();

        leftSpike  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-37.43, -43.15), Math.toRadians(103.54))
                .lineToConstantHeading(new Vector2d(-28.73, -58.72))
                .lineToConstantHeading(new Vector2d(23.01, -59.87))
                .splineToConstantHeading(new Vector2d(47.96, -29.80), Math.toRadians(0.00))
                .build();
    /*.splineTo(new Vector2d(-43.00, -35.00), Math.toRadians(135.00))
                .lineToSplineHeading(new Pose2d(-36.0, -59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(54.00, -58.00, Math.toRadians(180)))
                .build();*/

        centerSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.52, -41.09), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-28.73, -58.95), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(23.00, -60.00))
                .splineTo(new Vector2d(48.19, -36.52), Math.toRadians(0.00))
                .build();
                /*.splineToLinearHeading(new Pose2d(-36.00, -34.00, Math.toRadians(90.61)), Math.toRadians(90.61))
                .lineToSplineHeading(new Pose2d(-35.50, -60.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(54.00, -58.00, Math.toRadians(180)))
                .build();*/

        rightSpike  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-35.60, -38.58), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-28.73, -58.95), Math.toRadians(0.00))
                .lineToConstantHeading(new Vector2d(23.00, -60.00))
                .splineTo(new Vector2d(48.42, -42.93), Math.toRadians(0.00))
                .build();
                /*.splineToLinearHeading(new Pose2d(-30.00, -36.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                .lineToSplineHeading(new Pose2d(-40.00, -59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(55.00, -58.00, Math.toRadians(180)))
                .build();*/

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.mecanumDriveSubsystem.followTrajectorySequence(RedLeftPark);
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
