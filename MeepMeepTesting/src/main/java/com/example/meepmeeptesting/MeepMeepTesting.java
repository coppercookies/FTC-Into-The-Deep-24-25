package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //Code goes in here
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61.5, Math.toRadians(180)))
//drop specimen 1
                        .waitSeconds(4)
                        .strafeToConstantHeading(new Vector2d(10,-31.5))
                        .waitSeconds(1)
                        .strafeToConstantHeading(new Vector2d(10,-33))

                //pick and move block 1
                        .strafeToLinearHeading(new Vector2d(35,-45),Math.toRadians(25))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(315))
                        .waitSeconds(1)

                //pick and move block 2
                        .strafeToLinearHeading(new Vector2d(55,-45),Math.toRadians(15))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(345))
                        .waitSeconds(1)
                        .build()
        );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}