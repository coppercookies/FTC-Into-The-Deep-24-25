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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61.5, Math.toRadians(180)))
                .waitSeconds(3)
                .strafeToConstantHeading(new Vector2d(0,-32))
                .waitSeconds(5)

                .strafeToLinearHeading(new Vector2d(34,-47),Math.toRadians(360))
                .waitSeconds(2)
                //Be above the block
                .strafeToConstantHeading(new Vector2d(38,-10))
                .waitSeconds(2)
                .lineToX(47)


                .strafeToConstantHeading(new Vector2d(45,-61.5))
                .waitSeconds(4)
                .strafeToConstantHeading(new Vector2d(45,-50))


                .strafeToLinearHeading(new Vector2d(0, -32),Math.toRadians(180))
                .waitSeconds(5)

                .build()
        );

        mm.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}