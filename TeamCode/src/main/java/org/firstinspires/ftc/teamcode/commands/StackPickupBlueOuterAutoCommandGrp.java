package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class StackPickupBlueOuterAutoCommandGrp extends SequentialCommandGroup {
    public RobotBase robotBase;
    public Pose2d cyclePose;
    public StackPickupBlueOuterAutoCommandGrp(RobotBase robotBaseConst, Pose2d poseConst) {
        robotBase = robotBaseConst;
        TrajectorySequence BlueStack;

        cyclePose = poseConst;
        BlueStack = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(cyclePose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(25, 60),  Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-108, 60), Math.toRadians(180))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-98, 60), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(25, 60), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(45, 36),  Math.toRadians(90))
                .build();
    }
}
