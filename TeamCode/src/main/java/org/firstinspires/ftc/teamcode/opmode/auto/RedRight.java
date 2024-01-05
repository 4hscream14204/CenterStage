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

import java.util.Vector;


@Autonomous(name = "RedRight")
public class RedRight extends OpMode {

    public RobotBase robotBase;
    private TrajectorySequence LeftSpike;
    private TrajectorySequence MiddleSpike;
    private TrajectorySequence RightSpike;
    private TrajectorySequence InnerPark;
    private TrajectorySequence OuterPark;
    private TrajectorySequence parkLocation;
    public Pose2d startPose;
    public GamepadEx autoChassisController;

    @Override
    public void init(){
        autoChassisController = new GamepadEx(gamepad1);
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        telemetry.update();
        startPose = new Pose2d(15.00, -63.00, Math.toRadians(90.00));
        LeftSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(10.2, 45.28), Math.toRadians(135.00))
                .splineTo(new Vector2d(48.51, 28.99), Math.toRadians(90.00))
                //Need drop off code


/*
                .lineToLinearHeading(new Pose2d(17.00, -46.00, Math.toRadians(90.00)))

                .splineToLinearHeading(new Pose2d(40.00, -31.70, Math.toRadians(180.00)), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(52.00, -31.70, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> robotBase.Grabber.Drop())
                .lineTo(new Vector2d(51.00, -31.70))
                .lineTo(new Vector2d(52.00,-31.70))
                .waitSeconds(0.25)
                //.addTemporalMarker(() -> robotBase.grabber.downPosition())
                .lineToLinearHeading(new Pose2d(50.00, -31.70, Math.toRadians(180.00)))
                //.waitSeconds(1)
                .lineToConstantHeading(new Vector2d(45.00, -36.00))
                //.splineTo(new Vector2d(58.00, -61.0), Math.toRadians(0.00))

 */
                .build();

        MiddleSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.00, -41.00), Math.toRadians(95.44))
                .splineTo(new Vector2d(28.70, -38.24), Math.toRadians(-8.24))
                .splineToConstantHeading(new Vector2d(48.81, -36.48), Math.toRadians(0.00))
                //Add drop off code
                .build();


/*
                .splineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(12.00, -37.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(35.00, -38.00, Math.toRadians(180.00)))
                //.lineToLinearHeading(new Pose2d(50.50, -37.50, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.Drop();})
                .lineToSplineHeading(new Pose2d(51.50, -38.00, Math.toRadians(180.00)))
                //.waitSeconds(3)
                //.lineToLinearHeading(new Pose2d(40.00, -37.00, Math.toRadians(180.00)))
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(50.50, -38.00))
                .lineTo(new Vector2d(51.50,-38.00))
                .waitSeconds(0.5)
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.waitSeconds(1)
                .lineToLinearHeading(new Pose2d(45.00, -36.00, Math.toRadians(180.00)))
                //.splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

        RedRightCenterInner2 = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(RedRightCenterInner.end())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(50.50, -38.00))
                .lineTo(new Vector2d(51.50,-38.00))
                .waitSeconds(0.5)
                //.addDisplacementMarker(() -> {robotBase.Grabber.DownPosition();})
                //.waitSeconds(1)
                .lineToLinearHeading(new Pose2d(45.00, -36.00, Math.toRadians(180.00)))
                //.splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build();

 */

        RightSpike = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(new Pose2d(15.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.00, -41.00), Math.toRadians(95.44))
                .splineTo(new Vector2d(28.70, -38.24), Math.toRadians(-8.24))
                .splineToConstantHeading(new Vector2d(48.81, -36.48), Math.toRadians(0.00))
                .build();


                /*
                .splineToSplineHeading(new Pose2d(20.00, -37.00, Math.toRadians(60.00)), Math.toRadians(60.00))
                .lineToSplineHeading(new Pose2d(20.00, -43.00, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(43.00, -43.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(52.00, -43.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(51.00, -43.00))
                .lineTo(new Vector2d(52.00,-43.00))
                .waitSeconds(0.25)
                //.addTemporalMarker(() -> robotBase.Grabber.DownPosition())
                .lineToSplineHeading(new Pose2d(45.00, -36.00, Math.toRadians(180.00)))
                //.splineTo(new Vector2d(59.00, -58.00), Math.toRadians(0.00))
                //.splineToSplineHeading(new Pose2d(59.00, -58.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

                 */

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
            //robotBase.grabber.drop();
            //robotBase.mecanumDrive.followTrajectorySequence(RedRightCenterInner2);
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
    }
}
