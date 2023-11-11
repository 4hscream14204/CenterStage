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

import java.net.PortUnreachableException;

@Autonomous(name = "BlueRight")
public class BlueRight extends OpMode {
    public RobotBase robotBase;
    private TrajectorySequence BlueRightPark;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));
        BlueRightPark = robotBase.MecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, 63.30, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-38.35, 12.00, Math.toRadians(0.00)))
                .waitSeconds(20)
                .lineTo(new Vector2d(6.00, 12.00))
                .lineTo(new Vector2d(50.00, 12.00))
                .build();

        robotBase.MecanumDrive.setPoseEstimate(startPose);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        robotBase.MecanumDrive.followTrajectorySequence(BlueRightPark);
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
