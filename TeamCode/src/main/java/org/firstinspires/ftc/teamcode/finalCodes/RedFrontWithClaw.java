package org.firstinspires.ftc.teamcode.finalCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "RedFrontWithClaw")
public class RedFrontWithClaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(-40, -62, Math.toRadians(180));
        Pose2d beginPose = new Pose2d(-38, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx arm;
        DcMotorEx armSlider;
        CRServoImplEx RServo;
        CRServoImplEx LServo;
        ServoImplEx pivotServo;
        Servo armClaw;
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armSlider = hardwareMap.get(DcMotorEx.class, "armSlider");
        armClaw = hardwareMap.get(Servo.class, "armClaw");


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int zeroArmSliderPos = armSlider.getCurrentPosition();


        Action test = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-39,-61.5))
                .build();
        Action wait1Second = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .build();
        Action wait05Second = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .build();

        Action moveToBasket = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(228))
                .build();
        Action moveForwardBasket = drive.actionBuilder(new Pose2d(-52.5,-52.5,Math.toRadians(228)))
                .strafeTo(new Vector2d(-53.5,-53.5))
                .build();
        Action moveBackBasket = drive.actionBuilder(new Pose2d(-53.5,-53.5,Math.toRadians(228)))
                .strafeTo(new Vector2d(-50,-50))
                .build();


        Action turnToBlock1 = drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(228)))
                .strafeToLinearHeading(new Vector2d(-42.5, -38), Math.toRadians(90),new TranslationalVelConstraint(60))
                .build();
//        Action pickBlock1 = drive.actionBuilder(new Pose2d(-42.5, -43, Math.toRadians(90)))
//                .waitSeconds(1)
//                .strafeToConstantHeading(new Vector2d(-45, -35), new TranslationalVelConstraint(6))
//                .build();

        Action moveToBasket2 = drive.actionBuilder(new Pose2d(-45,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(228))
                .build();

        Action turnToBlock2 = drive.actionBuilder(new Pose2d(-52.5, -52.5, Math.toRadians(228)))
                .strafeToLinearHeading(new Vector2d(-57, -43), Math.toRadians(90))
                .build();
        Action pickBlock2 = drive.actionBuilder(new Pose2d(-57, -43, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-57, -35), new TranslationalVelConstraint(6))
                .build();
        Action moveToBasket3 = drive.actionBuilder(new Pose2d(-57,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                .build();
        Action faceForward = drive.actionBuilder(new Pose2d(-52,-52,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-40,-20),Math.toRadians(90))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                    new ArmAction(arm, armSlider, 75,0.2),
                    wait1Second,
                    new ArmClawAction(armClaw,0.67)

                )
        );
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                moveToBasket,
                                new ArmAction(arm,armSlider, 1080, 1)

                        ),
                        new SequentialAction(
                               new ArmSliderAction(armSlider,2000,1),
                               moveForwardBasket,
                               new ArmAction(arm, armSlider,-100,1),
                               new ArmClawAction(armClaw,0),
                               new ArmAction(arm, armSlider,120,1),
                               wait1Second,
                               moveBackBasket,

                               new ArmSliderAction(armSlider, -1990,1),
                               wait1Second,
                               new ArmAction(arm, armSlider, -1100,0.4),
                                wait1Second,
                                turnToBlock1
                        )

//                new ParallelAction(
//                    new ArmAction(arm,armSlider, 2080, 1),
//                    test
//                    //new ArmSliderAction(armSlider,2000,1),
//                    //new ArmSliderAction(armSlider, -1300,1),
//
//                    //new ArmAction(arm, armSlider, -1150,0.9),
//                )

//                        new SequentialAction(
//                                new ArmClawAction(armClaw,1)
//                        ),
//                        new ParallelAction(
//                                moveToBasket2,
//                                new ArmAction(arm,armSlider, 950, 1)
//                        ),
//                        new SequentialAction(
//                                new ArmSliderAction(armSlider,685,1),
//                                new ArmClawAction(armClaw,1),
//                                new ArmSliderAction(armSlider, -685,1)
//                        )


                )
        );
    }

    public class ArmAction implements Action {
        DcMotorEx arm;
        DcMotorEx armSlider;
        double armPower;
        int armPos;
        boolean initialized;
        int startArmPos;
        int armEndPos;
        boolean goingUp;

        public ArmAction(DcMotorEx arm, DcMotorEx armSlider, int armPos, double armPower) {
            this.arm = arm;
            this.armPower = armPower;
            this.armPos = armPos;
            this.initialized = false;
            this.startArmPos = 0;
            this.armEndPos = 0;
            this.armSlider = armSlider;
            if (armPos > 0)
                this.goingUp = false;
            else
                this.goingUp = true;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                startArmPos = arm.getCurrentPosition();
                armEndPos = armPos + startArmPos;
                arm.setTargetPosition(armEndPos);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                initialized = true;
            }

            if (goingUp) {
                if (arm.getCurrentPosition() <= armEndPos) {
                    return true;
                }
            } else {
                if (arm.getCurrentPosition() >= armEndPos) {
                    return true;
                }
            }

            if (armSlider.getCurrentPosition() > 100) {
                arm.setPower(0.7);//0.35
            } else {
                arm.setPower(0.25);
            }
            return false;
        }
    }

    public class ArmSliderAction implements Action {
        DcMotorEx armSlider;
        double armSliderPower;
        int armSliderPos;
        boolean initialized;
        int startArmSliderPos;
        int armSliderEndPos;
        boolean goingOut;

        public ArmSliderAction(DcMotorEx armSlider, int armSliderPos, double armSliderPower) {
            this.armSlider = armSlider;
            this.armSliderPower = armSliderPower;
            this.armSliderPos = armSliderPos;
            this.initialized = false;
            this.startArmSliderPos = 0;
            this.armSliderEndPos = 0;
            if (armSliderPos > 0)
                this.goingOut = true;
            else
                this.goingOut = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                startArmSliderPos = armSlider.getCurrentPosition();
                armSliderEndPos = armSliderPos + startArmSliderPos;
                armSlider.setTargetPosition(armSliderEndPos);
                armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlider.setPower(armSliderPower);
                initialized = true;
            }

            if (goingOut) {
                if (armSlider.getCurrentPosition() <= armSliderEndPos) {
                    return true;
                }
            } else {
                if (armSlider.getCurrentPosition() >= armSliderEndPos) {
                    return true;
                }
            }

            armSlider.setPower(0);
            return false;
        }
    }


    public class ArmClawAction implements Action {
        Servo armClaw;
        double armClawPos;
        ElapsedTime timer;

        public ArmClawAction(Servo armClaw, double armClawPos) {
            this.armClaw = armClaw;
            this.armClawPos = armClawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                armClaw.setPosition(armClawPos);
            }

            if (timer.seconds() < 0.4) {
                return true;
            } else {
                return false;
            }
        }
    }

}