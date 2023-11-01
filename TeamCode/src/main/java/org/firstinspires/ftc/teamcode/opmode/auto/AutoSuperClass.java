package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class AutoSuperClass extends OpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    RobotBase robotBase = new RobotBase(hardwareMap);



    @Override
    public void init() {
        //RedLeft
        Pose2d redLeftStart = new Pose2d(-40.29, -63.05, Math.toRadians(90));

        drive.setPoseEstimate(redLeftStart);


        TrajectorySequence redLeft1 = drive.trajectorySequenceBuilder(redLeftStart)

                .splineToSplineHeading(new Pose2d(-35.89, -34.42, Math.toRadians(90)), Math.toRadians(90))

                .build();

        //RedRight
        Pose2d redRightStart = new Pose2d(-39, -63, Math.toRadians(270));

        drive.setPoseEstimate(redRightStart);


        TrajectorySequence redRight1 = drive.trajectorySequenceBuilder(redRightStart)

                .build();

        //BlueLeft
        Pose2d blueLeftStart = new Pose2d(-39, -63, Math.toRadians(270));

        drive.setPoseEstimate(blueLeftStart);


        TrajectorySequence blueLeft1 = drive.trajectorySequenceBuilder(blueLeftStart)

                .build();

        //BlueRight
        Pose2d blueRightStart = new Pose2d(-39, -63, Math.toRadians(270));

        drive.setPoseEstimate(blueRightStart);


        TrajectorySequence blueRight1 = drive.trajectorySequenceBuilder(blueRightStart)

                .build();
    }
@Override
    public void init_loop(){}



    HuskyLens.Block[] blocks = robotBase.huskyLens.blocks();


    @Override
    public void start() {
    }

    @Override
    public void loop() {

    }
}
