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
    private TrajectorySequence BaRsLsTcIp;
    public Pose2d startPose;

    @Override
    public void init(){
        robotBase = new RobotBase(hardwareMap);
        robotBase.alliance = RobotBase.Alliance.RED;
        robotBase.startPosition = RobotBase.StartPosition.LEFT;
        startPose = new Pose2d(-38.35, 63.3, Math.toRadians(270.00));
  /*      BaRsLsTcIp = robotBase.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-38.25, 63.30, Math.toRadians(270.00)))
                .splineToSplineHeading(new Pose2d(-30.00, 36.00, Math.toRadians(315.00)), Math.toRadians(315.00))
                //.lineToSplineHeading(new Pose2d(-36.00, 41.00, Math.toRadians(270.00)))
                //.lineToSplineHeading(new Pose2d(-36.00, 64.00, Math.toRadians(360.00)))
                //.splineToSplineHeading(new Pose2d(36.00, 64.00, Math.toRadians(360.00)), Math.toRadians(360.00))
                //.splineToSplineHeading(new Pose2d(48.00, 42.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                //.splineToSplineHeading(new Pose2d(51.00, 42.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                //.splineToSplineHeading(new Pose2d(42.00, 42.00, Math.toRadians(180.00)), Math.toRadians(180.00))
               //.splineToSplineHeading(new Pose2d(55.00, 13.00, Math.toRadians(360.00)), Math.toRadians(360.00))
                .build();
*/
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
        robotBase.mecanumDrive.followTrajectorySequence(BlueRightPark);
     /*   if (robotBase.propPosition == RobotBase.PropPosition.MIDDLE) {
            robotBase.mecanumDrive.followTrajectorySequence(BaRsLsTcIp);
            robotBase.grabber.drop();
            robotBase.mecanumDrive.followTrajectorySequence(BaRsLsTcIp);
        } else if (robotBase.propPosition == RobotBase.PropPosition.RIGHT) {
            robotBase.mecanumDrive.followTrajectorySequence(BaRsLsTcIp);
        } else {
            robotBase.mecanumDrive.followTrajectorySequence(BaRsLsTcIp);
        } */
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
