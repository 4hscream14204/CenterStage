package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, 63.3, Math.toRadians(270)))
                                .splineTo(new Vector2d(-33.00, 37.00), Math.toRadians(-45))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-36.00, 60.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(12.00, 60.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(45.00, 36.00), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(50.00, 36.00), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineTo(new Vector2d(40.00, 36.00), Math.toRadians(0.00))
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}