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
    private TrajectorySequence leftSpike;
    private TrajectorySequence rightSpike;
    private TrajectorySequence centerSpike;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.BLUE;
        robotBase.startPosition = RobotBase.StartPosition.RIGHT;
        startPose = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));

       leftSpike = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, 63.3, Math.toRadians(270.00)))
               .splineToLinearHeading(new Pose2d(-30.00, 36.00, Math.toRadians(315.00)), Math.toRadians(315.00))
               .lineToLinearHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)))
               .splineToLinearHeading(new Pose2d(-38.00, 59.00, Math.toRadians(180.00)),Math.toRadians(90.00))
               .lineTo(new Vector2d(55.00, 60.00))
              // .splineToLinearHeading(new Pose2d(55, 60, Math.toRadians(180.00)), Math.toRadians(0.00))
               .addTemporalMarker(() -> {robotBase.grabber.dropPosition();})
               .waitSeconds(1)
               .addTemporalMarker(() -> {robotBase.grabber.drop();})
               .waitSeconds(1)
               .addTemporalMarker(() -> {robotBase.grabber.downPosition();})

               /*
               .splineToSplineHeading(new Pose2d(48.00, 42.00, Math.toRadians(180.00)), Math.toRadians(360.00))
               .splineToSplineHeading(new Pose2d(51.00, 42.00, Math.toRadians(180.00)), Math.toRadians(180.00))
               .splineToSplineHeading(new Pose2d(42.00, 42.00, Math.toRadians(180.00)), Math.toRadians(180.00))
               .splineToSplineHeading(new Pose2d(55.00, 13.00, Math.toRadians(360.00)), Math.toRadians(360.00))
                */
               .build();

        centerSpike = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-37.42, 63.30, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-36.00, 34.00, Math.toRadians(269.39)), Math.toRadians(269.39))
                .lineToSplineHeading(new Pose2d(-38.00, 59.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(55.00, 59.00))
                .addTemporalMarker(() -> {robotBase.grabber.dropPosition();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})
                /*
                .lineToSplineHeading(new Pose2d(50.00, 36.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(35.00, 36.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(60.00, 13.00), Math.toRadians(360.00))

                 */
                .build();

        rightSpike = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, 63.30, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-43.00, 35.00, Math.toRadians(225.00)), Math.toRadians(225.00))
                .lineToSplineHeading(new Pose2d(-32.50, 59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(55.00, 59.00, Math.toRadians(180)))
                //.lineTo(new Vector2d(55.00, 59.00))
                .addTemporalMarker(() -> {robotBase.grabber.dropPosition();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.drop();})
                .waitSeconds(1)
                .addTemporalMarker(() -> {robotBase.grabber.downPosition();})


                /*
                .lineToConstantHeading(new Vector2d(40.00, 29.50))
                .splineToSplineHeading(new Pose2d(50.00, 29.50, Math.toRadians(180.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(45.00, 29.50), Math.toRadians(180.00))
                .splineTo(new Vector2d(60.00, 12.00), Math.toRadians(360.00))

                 */
                .build();


        BlueRightPark = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.35, 63.30, Math.toRadians(270.00)))
                //.waitSeconds(20)
                .splineTo(new Vector2d(-38.35, 13.50), Math.toRadians(0.00))
                .waitSeconds(20)
                .lineTo(new Vector2d(6.00, 12.00))
                .lineTo(new Vector2d(50.00, 12.00))
                .build();

        robotBase.mecanumDrive.setPoseEstimate(startPose);
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
        //robotBase.mecanumDrive.followTrajectorySequence(BlueRightPark);
      /*  if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDrive.followTrajectorySequence(centerSpike);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDrive.followTrajectorySequence(rightSpike);
        } else {
            robotBase.mecanumDrive.followTrajectorySequence(leftSpike);
        } */
        robotBase.mecanumDrive.followTrajectorySequence(rightSpike);
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
