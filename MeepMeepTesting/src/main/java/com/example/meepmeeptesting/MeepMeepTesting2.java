package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
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

                        .strafeToLinearHeading(new Vector2d(45,-45),Math.toRadians(315))

                        //pick up specimen 1
                        .strafeToLinearHeading(new Vector2d(45,-45),Math.toRadians(360))
                        .strafeToConstantHeading(new Vector2d(31.3,-58.2))

                        .build()
        );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}