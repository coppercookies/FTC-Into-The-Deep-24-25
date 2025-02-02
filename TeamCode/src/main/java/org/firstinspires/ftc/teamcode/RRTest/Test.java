package org.firstinspires.ftc.teamcode.RRTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.jvm.internal.TypeParameterReference;


@Autonomous (name = "Test")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(-40, -62, Math.toRadians(180));
        Pose2d beginPose = new Pose2d(14.4, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        waitForStart();



        Actions.runBlocking(drive.actionBuilder(beginPose)

                        .strafeToConstantHeading(new Vector2d(10,-31.5))
                        .waitSeconds(1)

                        .strafeToConstantHeading(new Vector2d(10,-33))



//                //Go to block 1
                        .splineToLinearHeading(new Pose2d(23,-40,Math.toRadians(90)),Math.toRadians(360))
                        .splineToLinearHeading(new Pose2d(46,-5,Math.toRadians(90)),Math.toRadians(360))
                        .strafeToLinearHeading(new Vector2d(46,-57),Math.toRadians(90))
                        .waitSeconds(0.01)

                        //Go to block 2

                        .splineToConstantHeading(new Vector2d(46, -5), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(58, -5), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(270.00))




                        //pick up specimen 1
                        .strafeToLinearHeading(new Vector2d(45,-45),Math.toRadians(360))
                        .strafeToConstantHeading(new Vector2d(31.3,-58.2))
                        .waitSeconds(0.7)


                        //drop specimen 2 and come back
                        .strafeToLinearHeading(new Vector2d(4, -30.6),Math.toRadians(180))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(31.6, -54),Math.toRadians(360))
                        .strafeTo(new Vector2d(31.6,-57.6))
                        .waitSeconds(1)


                        //drop specimen 3 and come back
                        .strafeToLinearHeading(new Vector2d(6, -30.6),Math.toRadians(180))
                        .waitSeconds(1)

                        .strafeToLinearHeading(new Vector2d(31.6, -54),Math.toRadians(360))
                        .strafeTo(new Vector2d(31.6,-57.6))
                        .waitSeconds(1)


                        //drop specimen 4 and park
                        .strafeToLinearHeading(new Vector2d(8, -30.6),Math.toRadians(180))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(47,-56),Math.toRadians(90), new TranslationalVelConstraint(100))

                        .build()

        );
    }

}