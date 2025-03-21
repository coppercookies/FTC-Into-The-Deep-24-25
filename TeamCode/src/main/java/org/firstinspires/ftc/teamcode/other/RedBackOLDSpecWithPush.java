package org.firstinspires.ftc.teamcode.other;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Disabled
@Autonomous (name = "RedBack 3 Spec and Push")
public class RedBackOLDSpecWithPush extends LinearOpMode {

    //Claw Class try
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(0.7);
//                return false;
//            }
//        }
//        public Action closeClaw() {
//            return new CloseClaw();
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(0);
//                return false;
//            }
//        }
//        public Action openClaw() {
//            return new OpenClaw();
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(9, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx clawSlider;

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(45),
                new AngularVelConstraint(Math.toRadians(180))
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-40, 40);


        ServoImplEx claw;

        claw = hardwareMap.get(ServoImplEx.class, "claw");

        clawSlider = hardwareMap.get(DcMotorEx.class, "clawSlider");
        clawSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Actions.runBlocking(
                new ClawAction(claw, 0.7)
        );


        Action moveToSub = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(0,-31.5))
                .build();

        Action moveToBlock = drive.actionBuilder(new Pose2d(0,-31.5,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(34,-47),Math.toRadians(360),baseVelConstraint,baseAccelConstraint)
//                .waitSeconds(2)
                //Be above the block
                .strafeToConstantHeading(new Vector2d(38,-5),baseVelConstraint,baseAccelConstraint)
                //1/6
                //.waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(48,-5),baseVelConstraint,baseAccelConstraint)
                .build();

        Action pushBlock = drive.actionBuilder(new Pose2d(48,-5,Math.toRadians(360)))
                .strafeToConstantHeading(new Vector2d(48,-54))//57
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(48,-45))
//                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(31.3,-58.2))

                //.waitSeconds(2)
                .build();
//
        Action moveToSub2 = drive.actionBuilder(new Pose2d(31.3,-58.2,Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(4, -30.6),Math.toRadians(180))
                .build();

        Action moveToPickup2 = drive.actionBuilder(new Pose2d(4,-30.6,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(31.6, -54),Math.toRadians(360))
                .strafeTo(new Vector2d(31.6,-57.6))
                .build();
        Action moveToSub3 = drive.actionBuilder(new Pose2d(31.6,-57.6,Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(6, -30.6),Math.toRadians(180))
                .build();

        Action pushBlock2 = drive.actionBuilder(new Pose2d(6,-30.6,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(40,-44),Math.toRadians(360),baseVelConstraint,baseAccelConstraint)

                .strafeToConstantHeading(new Vector2d(45,-5),baseVelConstraint,baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(58,-5),baseVelConstraint,baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(58,-55),new TranslationalVelConstraint(80),baseAccelConstraint)
                .build();




        waitForStart();

        Actions.runBlocking(new SequentialAction(

                        new ParallelAction(
                                moveToSub,
                                new ClawSliderAction(clawSlider,-2100,1)
                        ),

                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 280, 0.8),
                                new PatientClawAction(claw, 0.0)
                        ),

                        new ParallelAction(
                                moveToBlock,
                                new ClawSliderAction(clawSlider,1395,0.8)
                        ),

                        new SequentialAction(
                                pushBlock,
                                //The wait is in pushBlock itself
                                new PatientClawAction(claw,0.7)
                        ),

                        new ParallelAction(
                                moveToSub2,
                                new ClawSliderAction(clawSlider,-1700,0.5)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 315, 0.8),
                                new PatientClawAction(claw, 0.0)
                        ),
                        new ParallelAction(
                                moveToPickup2,
                                new ClawSliderAction(clawSlider, 1395, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7),
                                new ClawSliderAction(clawSlider, -400, 0.8)
                        ),
                        new ParallelAction(
                                moveToSub3,
                                new ClawSliderAction(clawSlider, -1300, 0.8)

                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 315, 0.8),
                                new PatientClawAction(claw, 0.0)

                        ),
                        new ParallelAction(
                                pushBlock2,
                                new ClawSliderAction(clawSlider,1780,0.8 )
                        )



                )
        );

        ///////////////////////////////
//        .stopAndAdd(new ClawSliderAction(clawSlider,1000,-0.3))
//                .waitSeconds(2)
//
//                .strafeToLinearHeading(new Vector2d(34,-47),Math.toRadians(360))
//                .waitSeconds(2)
//                //Be above the block
//                .strafeToConstantHeading(new Vector2d(38,-10))
////                        .waitSeconds(2)
//                .lineToX(47)
//
//
//                .strafeToConstantHeading(new Vector2d(45,-61.5))
//                .waitSeconds(4)
//                .strafeToConstantHeading(new Vector2d(45,-50))
//
//
//                .strafeToLinearHeading(new Vector2d(0, -32),Math.toRadians(180))
//                .waitSeconds(5)
//
//                .build()
//////////////////////////////////////
//                        .stopAndAdd(new ClawAction(claw, 0.5))
//
//                        .strafeToLinearHeading(new Vector2d(34,-47),Math.toRadians(360))
//                        .waitSeconds(2)
//                        //Be above the block
//                        .strafeToConstantHeading(new Vector2d(38,-10))
//                        .waitSeconds(2)
//                        .lineToX(47)
//
//
//                        .strafeToConstantHeading(new Vector2d(45,-61.5))
//                        .waitSeconds(4)
//                        .strafeToConstantHeading(new Vector2d(45,-50))
//
//
//                        .strafeToLinearHeading(new Vector2d(0, -32),Math.toRadians(180))
//                        .waitSeconds(5)





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

//            if (armSlider.getCurrentPosition() != armSliderPos) {
//                return true;
//            }

            armSlider.setPower(0);
            return false;
        }
    }

    //    public class IntakeAction implements Action {
//        CRServo RServo;
//        CRServo LServo;
//        CRServo topTake;
//        double intakePower;
//
//        public IntakeAction(CRServo RServo,CRServo LServo, CRServo topTake, double intakePower) {
//            this.RServo = RServo;
//            this.LServo = LServo;
//            this.topTake = topTake;
//            this.intakePower = intakePower;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            RServo.setPower(intakePower);
//            LServo.setPower(-intakePower);
//            topTake.setPower(intakePower);
//            return false;
//        }
//    }
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

            if (timer.seconds() < 0.25) {
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

            if (goingUp)
            {
                if (clawSlider.getCurrentPosition() >= clawSlideEndPos) {
                    return true;
                }
            }
            else
            {
                if (clawSlider.getCurrentPosition() <= clawSlideEndPos) {
                    return true;
                }
            }
//            if (armSlider.getCurrentPosition() != armSliderPos) {
//                return true;
//            }
            //Negative goes up
            clawSlider.setPower(-0.15);
            return false;
        }
    }
}