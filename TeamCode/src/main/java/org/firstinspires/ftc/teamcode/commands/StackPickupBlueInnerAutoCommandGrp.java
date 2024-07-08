package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class StackPickupBlueInnerAutoCommandGrp extends SequentialCommandGroup {
    public RobotBase robotBase;
    public Pose2d cyclePose;
    public StackPickupBlueInnerAutoCommandGrp(RobotBase robotBaseConst, Pose2d poseConst) {
        robotBase = robotBaseConst;
        TrajectorySequence BlueStack;

        cyclePose = poseConst;
        BlueStack = robotBase.mecanumDriveSubsystem.trajectorySequenceBuilder(cyclePose)
                .splineToConstantHeading(new Vector2d(25, 9),  Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-90, 12, Math.toRadians(0.00)), Math.toRadians(180.00))
                .build();
    }
}
