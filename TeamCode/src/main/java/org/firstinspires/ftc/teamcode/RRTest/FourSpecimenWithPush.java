package org.firstinspires.ftc.teamcode.RRTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
@Disabled

@Autonomous (name = "ForwardPush4Spec")
public class FourSpecimenWithPush extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(14.4, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");
        DcMotorEx clawSlider = hardwareMap.get(DcMotorEx.class, "clawSlider");
        clawSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Init Code
        Actions.runBlocking(
                new ClawAction(claw, 0.73)
        );

        Action moveToSub1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(15,-31.5))
                .build();

        Action moveToBlockAndPush = drive.actionBuilder(new Pose2d(15,-31.5,Math.toRadians(180)))

                .strafeToConstantHeading(new Vector2d(15,-34))
                .splineToLinearHeading(new Pose2d(30,-40,Math.toRadians(270)),Math.toRadians(360))
                .splineToLinearHeading(new Pose2d(47,-10,Math.toRadians(270)),Math.toRadians(360))
                .strafeToLinearHeading(new Vector2d(47,-57),Math.toRadians(270))
                .build();

        Action pushBlock2 = drive.actionBuilder(new Pose2d(47,-57,Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47,-10))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(57, -10), Math.toRadians(-90))
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(57, -57), Math.toRadians(270))
                .build();

        Action moveToPickup1 = drive.actionBuilder(new Pose2d(59,-57,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(31, -54),Math.toRadians(370))
//                .strafeToConstantHeading(new Vector2d(31, -54))
                .strafeToConstantHeading(new Vector2d(31,-63))
                .build();

        Action moveToSub2 = drive.actionBuilder(new Pose2d(31,-63,Math.toRadians(370)))
                .strafeToLinearHeading(new Vector2d(12, -32),Math.toRadians(180))
                .build(); //9

        Action moveToPickup2 = drive.actionBuilder(new Pose2d(12, -32, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(31, -55),Math.toRadians(370))
                .strafeTo(new Vector2d(31,-63))
                .build();

        Action moveToSub3 = drive.actionBuilder(new Pose2d(31, -63, Math.toRadians(370)))
                .strafeToLinearHeading(new Vector2d(3, -32),Math.toRadians(180))
                .build();//6

        Action moveToPickup3 = drive.actionBuilder(new Pose2d(3,-32,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(28.6, -55),Math.toRadians(370))
                .strafeTo(new Vector2d(31,-63))
                .build();

        Action moveToSub4 = drive.actionBuilder(new Pose2d(31, -63, Math.toRadians(370)))
                .strafeToLinearHeading(new Vector2d(-1, -32), Math.toRadians(180))
                .build(); //3

        Action park = drive.actionBuilder(new Pose2d(-1, -32, Math.toRadians(180)))
                .strafeTo(new Vector2d(2,-40))
                .strafeToLinearHeading(new Vector2d(45,-60),Math.toRadians(90), new TranslationalVelConstraint(130))
                .build();

// 2100-300-1375+1675-300-1375+1675-300-1375+1675-300-1820
//
//        Action turn = drive.actionBuilder((new Pose2d(4,-31.5,Math.toRadians(180))))
//                .strafeToLinearHeading(new Vector2d(3,-35),Math.toRadians(90))
//                .build();

        Action wait05Seconds = drive.actionBuilder((new Pose2d(-3,-34,Math.toRadians(90))))
                .waitSeconds(0.05)
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                moveToSub1,
                                new ClawSliderAction(clawSlider,-2100,1)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 325, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.4)
                        ),
                        new ParallelAction(
                                moveToBlockAndPush,
                                new ClawSliderAction(clawSlider,1370,0.8)
                        ),
                        new SequentialAction(
                                pushBlock2,
                                moveToPickup1,
                                new PatientClawAction(claw,0.73),
                                wait05Seconds
                        ),
////425
                        new ParallelAction(
                                moveToSub2,
                                new ClawSliderAction(clawSlider,-1695,0.9)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 325, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.4)
                        ),
                        //Third spec
                        new ParallelAction(
                                moveToPickup2,
                                new ClawSliderAction(clawSlider, 1370, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.73)
                        ),
                        new ParallelAction(
                                moveToSub3,
                                new ClawSliderAction(clawSlider, -1695, 0.9)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 325, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.4)
                        ),
                        //Fourth spec
                        new ParallelAction(
                                moveToPickup3,
                                new ClawSliderAction(clawSlider, 1375, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.73),
                                wait05Seconds
                        ),
                        new ParallelAction(
                                moveToSub4,
                                new ClawSliderAction(clawSlider, -1705, 0.9)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 330 , 0.8),
                                new PatientClawAction(claw, 0.4)
                        ),
                        new ParallelAction(
                                park,
                                new ClawSliderAction(clawSlider,1780,1)
                        )
                )
        );
    }

    public class ArmAction implements Action {
        DcMotor arm;
        double armPower;
        int armPos;
        boolean initialized;
        int startArmPos;
        public ArmAction(DcMotor arm, int armPos, double armPower) {
            this.arm = arm;
            this.armPower = armPower;
            this.armPos = armPos;
            initialized = false;
            startArmPos = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                startArmPos = arm.getCurrentPosition();
                arm.setTargetPosition(armPos - startArmPos);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                initialized = true;
            }

            if (arm.getCurrentPosition() < armPos) {
                return true;
            }
//            if (arm.getCurrentPosition() != armPos) {
//                return true;
//            }

            arm.setPower(0);
            return false;
        }
    }

    public class ArmSliderAction implements Action {
        DcMotor armSlider;
        double armSliderPower;
        int armSliderPos;
        boolean initialized;
        int startArmSliderPos;
        public ArmSliderAction(DcMotor armSlider, int armSliderPos, double armSliderPower) {
            this.armSlider = armSlider;
            this.armSliderPower = armSliderPower;
            this.armSliderPos = armSliderPos;
            initialized = false;
            startArmSliderPos = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                startArmSliderPos = armSlider.getCurrentPosition();
                armSlider.setTargetPosition(armSliderPos - startArmSliderPos);
                armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlider.setPower(armSliderPower);
                initialized = true;
            }

            if (armSlider.getCurrentPosition() < armSliderPos){
                return true;
            }


            armSlider.setPower(0);
            return false;
        }
    }

    public class IntakeAction implements Action {
        CRServo RServo;
        CRServo LServo;
        CRServo topTake;
        double intakePower;

        public IntakeAction(CRServo RServo,CRServo LServo, CRServo topTake, double intakePower) {
            this.RServo = RServo;
            this.LServo = LServo;
            this.topTake = topTake;
            this.intakePower = intakePower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            RServo.setPower(intakePower);
            LServo.setPower(-intakePower);
            topTake.setPower(intakePower);
            return false;
        }
    }
    ///////////////////////////////////////////////////////////////////////////
    public class ClawAction implements Action {
        ServoImplEx claw;
        double clawPos;

        public ClawAction(ServoImplEx claw, double clawPos) {
            this.claw = claw;
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(clawPos);
            return false;
        }
    }


    //////////////////////////////////////////////////////////////////////////
    public class PatientClawAction implements Action {
        ServoImplEx claw;
        double clawPos;
        ElapsedTime timer;

        public PatientClawAction(ServoImplEx claw, double clawPos) {
            this.claw = claw;
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                claw.setPosition(clawPos);
            }

            if (timer.seconds() < 0.2) {
                return true;
            } else {
                return false;
            }
        }
    }

    public class ClawSliderAction implements Action {
        DcMotor clawSlider;
        double clawSliderPower;
        int clawSliderPos;
        boolean initialized;
        int startClawSliderPos;
        int clawSlideEndPos;
        boolean goingUp;
        public ClawSliderAction(DcMotor clawSlider, int clawSliderPos, double clawSliderPower) {
            this.clawSlider = clawSlider;
            this.clawSliderPower = clawSliderPower;
            this.clawSliderPos = clawSliderPos;
            this.initialized = false;
            this.startClawSliderPos = 0;
            this.clawSlideEndPos = 0;
            if (clawSliderPos > 0)
                this.goingUp = false;
            else
                this.goingUp = true;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                startClawSliderPos = clawSlider.getCurrentPosition(); // -1200, 0
                clawSlideEndPos = clawSliderPos + startClawSliderPos; // 200 => -1000, -1200 => -1200
                clawSlider.setTargetPosition(clawSlideEndPos);
                clawSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawSlider.setPower(clawSliderPower);
                initialized = true;
            }

            if (goingUp) {
                if (clawSlider.getCurrentPosition() >= clawSlideEndPos) {
                    return true;
                }
            }
            else {
                if (clawSlider.getCurrentPosition() <= clawSlideEndPos) {
                    return true;
                }
            }

            //Negative goes up
            clawSlider.setPower(-0.15);
            return false;
        }
    }
}