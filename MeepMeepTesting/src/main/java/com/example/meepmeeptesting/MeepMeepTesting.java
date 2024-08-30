package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.font.TrueTypeFont;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.5,17.75)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(17.5, -63, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(18.00, -38.00, Math.toRadians(45.00)), Math.toRadians(45.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(24,-48, Math.toRadians(0)), Math.toRadians(330))
                                //.lineTo(new Vector2d(24,-48))
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(45, -41), Math.toRadians(0.00))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(50.00,-41.00))
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(40, -36))
                                .build());
        //Don't delete lines below
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}