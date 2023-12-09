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

    private TrajectorySequence leftSpike;

    private TrajectorySequence centerSpike;

    private TrajectorySequence rightSpike;
    private TrajectorySequence RedLeftMiddleOuter;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, -63.3, Math.toRadians(90.00));

        leftSpike  = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-43.00, -35.00), Math.toRadians(135.00))
                .lineToSplineHeading(new Pose2d(-36.0, -59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(54.00, -58.00, Math.toRadians(180)))
                        .addTemporalMarker(() -> {robotBase.grabber.backDropDrop();})
                        .waitSeconds(1)
                        .addTemporalMarker(() -> {robotBase.grabber.drop();})
                        .waitSeconds(1)
                        .addTemporalMarker(() -> {robotBase.grabber.downPosition();})

                /*
                .lineToConstantHeading(new Vector2d(40.00, -29.50))
                .splineToSplineHeading(new Pose2d(50.00, -29.50, Math.toRadians(180.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(45.00, -29.50), Math.toRadians(180.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))

                 */

                .build();

        centerSpike = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-36.00, -34.00, Math.toRadians(90.61)), Math.toRadians(90.61))
                .lineToSplineHeading(new Pose2d(-35.50, -60.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(54.00, -58.00, Math.toRadians(180)))
                .addTemporalMarker(() -> {robotBase.grabber.backDropDrop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})

                .build();

        rightSpike  = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, -63.30, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-30.00, -36.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                .lineToSplineHeading(new Pose2d(-40.00, -59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(55.00, -58.00, Math.toRadians(180)))
                .addTemporalMarker(() -> {robotBase.grabber.backDropDrop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})

                .build();

           /*
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

         */
    }
    @Override
    public void init_loop(){
        robotBase.propPosition = robotBase.huskyLensSubsystem.getLocation(robotBase.alliance, robotBase.startPosition);
        telemetry.addData("InitLoop","true");
        telemetry.addData("Detection",(robotBase.propPosition));
        telemetry.update();
    }
    @Override
    public void start(){
        robotBase.mecanumDrive.setPoseEstimate(startPose);
        //robotBase.mecanumDrive.followTrajectorySequence(RedLeftPark);
          if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDrive.followTrajectorySequence(centerSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.LEFT) {
            robotBase.mecanumDrive.followTrajectorySequence(leftSpike);
        } else {
            robotBase.mecanumDrive.followTrajectorySequence(rightSpike);
        }
       // robotBase.mecanumDrive.followTrajectorySequence(rightSpike);
       // robotBase.mecanumDrive.followTrajectorySequence(RedLeftMiddleOuter);
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