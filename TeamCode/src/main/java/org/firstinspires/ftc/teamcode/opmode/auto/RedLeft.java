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
    private TrajectorySequence RedLeftMiddleOuter;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        RedLeftPark  = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-38.35, -12.00, Math.toRadians(0.00)))
                .waitSeconds(20)
                .lineTo(new Vector2d(6.00, -12.00))
                .lineTo(new Vector2d(50, -12.00))
                .build();

        robotBase.mecanumDrive.setPoseEstimate(startPose);


        RedLeftMiddleOuter  = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-39.00, -63.30, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-35.17, -32.28, Math.toRadians(95.00)), Math.toRadians(95.00))
                .splineToSplineHeading(new Pose2d(-33.00, -37.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .lineToSplineHeading(new Pose2d(37.00, -37.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(51.00, -37.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(41.00, -37.00, Math.toRadians(180.00)))
                .splineToConstantHeading(new Vector2d(61.00, -12.00), Math.toRadians(0.00))
                .build();
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        //robotBase.mecanumDrive.followTrajectorySequence(RedLeftPark);
        robotBase.mecanumDrive.followTrajectorySequence(RedLeftMiddleOuter);
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
