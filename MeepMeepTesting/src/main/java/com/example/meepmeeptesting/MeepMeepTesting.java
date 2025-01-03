package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -61.5, Math.toRadians(180)))
                        .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-52, -52),Math.toRadians(225))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-48,-40),Math.toRadians(90))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-52, -52),Math.toRadians(225))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(225))
                .waitSeconds(4)
                .build()
        );

        mm.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}