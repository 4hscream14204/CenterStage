package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    private TrajectorySequence LeftSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence RightSpike;

    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private TrajectorySequence parkLocation;

    public GamepadEx autoChassisController;

    @Override
    public void init(){
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        telemetry.update();
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));
        LeftSpike  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-47.00, -38.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(-33.00, -55.00))
                .lineToLinearHeading(new Pose2d(-33.00, -21.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-33.00, -12.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(25.00, -12.00, Math.toRadians(180.00)))
                .splineToLinearHeading(new Pose2d(45.00, -27, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-36.00, -34.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineToLinearHeading(new Pose2d(-34, -43, Math.toRadians(45.00)))
                .splineToSplineHeading(new Pose2d(-5.00, -38.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .splineToSplineHeading(new Pose2d(45.00, -36.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .build();

        RightSpike  = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-28.00, -39.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                .lineToSplineHeading(new Pose2d(-40.00, -50.00, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-20.00, -12.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                //.waitSeconds(10)
                .splineTo(new Vector2d(36.04, -24.19), Math.toRadians(315.))
                .splineToSplineHeading(new Pose2d(45.00, -43.00, Math.toRadians(180.00)), Math.toRadians(270.00))
                .build();


        OuterPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(55.00, -62.00), Math.toRadians(0.00))
                .build();

        InnerPark = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(45.00, -36.00, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(55.00, -12.00), Math.toRadians(0.00))
                .build();

        robotBase.mecanumDriveSubsystem.setPoseEstimate(startPose);
        parkLocation = InnerPark;
        robotBase.parkSide = RobotBase.ParkSide.INNER;
    }
    @Override
    public void init_loop(){

        autoChassisController.readButtons();
        if (autoChassisController.wasJustPressed((GamepadKeys.Button.Y))) {
            if (robotBase.parkSide == RobotBase.ParkSide.INNER) {
                robotBase.parkSide = RobotBase.ParkSide.OUTER;
                parkLocation = OuterPark;
            } else {
                robotBase.parkSide = RobotBase.ParkSide.INNER;
                parkLocation = InnerPark;
            }
        }
        robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
        telemetry.addData("InitLoop","true");
        telemetry.addData("Detection",(robotBase.propPosition));
        telemetry.addData("Park Side", (robotBase.parkSide));
        telemetry.update();

    }
    @Override
    public void start(){

        if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(MiddleSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(RightSpike);
        } else {
            robotBase.mecanumDriveSubsystem.followTrajectorySequence(LeftSpike);
        }
        robotBase.mecanumDriveSubsystem.followTrajectorySequence(parkLocation);

    }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double dblCurrentHeading = angles.firstAngle;
        DataStorageSubsystem.dblIMUFinalHeading = dblCurrentHeading;
        DataStorageSubsystem.alliance = robotBase.alliance.RED;
    }
}
