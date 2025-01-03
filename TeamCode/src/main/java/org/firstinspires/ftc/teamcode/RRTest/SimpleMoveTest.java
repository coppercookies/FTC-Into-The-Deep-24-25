package org.firstinspires.ftc.teamcode.RRTest;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class SimpleMoveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-39, -65, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                        .waitSeconds(4)
                        //Go to first sample and go straight to pick up
                        .strafeToLinearHeading(new Vector2d(-50,-40),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-50,-30),Math.toRadians(90))
                        //Go to basket with first sample
                        .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                        .waitSeconds(4)
                        //Go to second sample and pick up
                        .strafeToLinearHeading(new Vector2d(-60,-40),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-60,-30),Math.toRadians(90))
                        //Go to basket with second sample
                        .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                        .waitSeconds(4)

                        //Go to third sample and pick up
                        .strafeToLinearHeading(new Vector2d(-60,-40),Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(-60,-30),Math.toRadians(135))
                        //Go to basket with third sample
                        .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                        .waitSeconds(4)
                        .build()
        );

        }
}
