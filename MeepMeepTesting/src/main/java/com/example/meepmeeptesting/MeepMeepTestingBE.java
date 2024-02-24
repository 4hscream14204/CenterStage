package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBE {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, -63.3, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-40, -39.11), Math.toRadians(135.00))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-36.00, -60.00), Math.toRadians(360))
                                //.addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new GrabAndWristEscapeCommandGrp(robotBase.leftWristSubsystem,
                                //        robotBase.leftClawSubsystem,
                                //        robotBase.armSubsystem)))
                                .splineToConstantHeading(new Vector2d(12.00, -60.00), Math.toRadians(360))
                                //.addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new DropOffPositionLowCommandGrp(robotBase.leftSlideSubsystem,
                                //        robotBase.armSubsystem,
                                //        robotBase.leftWristSubsystem,
                                //        robotBase.intakeSubsystem,
                                //        RobotBase.SlideHeight.LOWEST)))
                                .splineToConstantHeading(new Vector2d(45.00, -26.00), Math.toRadians(360.00))
                                .splineToConstantHeading(new Vector2d(51.00, -26.00), Math.toRadians(360.00))
                                //.addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new ClawOpenCommand(robotBase.armSubsystem,
                                //        robotBase.leftClawSubsystem)))
                                //.addDisplacementMarker(() -> CommandScheduler.getInstance().schedule(new UniversalGrabbingPosCommand(robotBase)))
                                .waitSeconds(0.5)
                                .lineTo(new Vector2d(40.00, -36.00))
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}