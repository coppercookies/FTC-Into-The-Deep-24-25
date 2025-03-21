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

@Autonomous (name = "OLD Red Front")
public class RedFront extends LinearOpMode {

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
        ServoImplEx claw;
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armSlider = hardwareMap.get(DcMotorEx.class, "armSlider");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        RServo = hardwareMap.get(CRServoImplEx.class, "RServo");
        LServo = hardwareMap.get(CRServoImplEx.class, "LServo");
        pivotServo = hardwareMap.get(ServoImplEx.class, "pivotServo");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int zeroArmSliderPos = armSlider.getCurrentPosition();

        Actions.runBlocking(
                new PivotServoAction(pivotServo, 0.85)
        );

        Action wait1Second = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .build();
        Action wait05Second = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .build();

        Action moveToBasket = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-49.5, -50), Math.toRadians(228))
                .build();

        Action turnToBlock1 = drive.actionBuilder(new Pose2d(-49.5, -50, Math.toRadians(228)))
                .strafeToLinearHeading(new Vector2d(-45, -43), Math.toRadians(90),
                        new TranslationalVelConstraint(60))
                .build();
        Action pickBlock1 = drive.actionBuilder(new Pose2d(-45, -43, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-45, -35), new TranslationalVelConstraint(6))
                .build();

        Action moveToBasket2 = drive.actionBuilder(new Pose2d(-45,-35,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                .build();

        Action turnToBlock2 = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
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



        waitForStart();

        RServo.setPwmEnable();
        LServo.setPwmEnable();
        RServo.setPower(0);
        LServo.setPower(0);

        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        moveToBasket,
                        new ArmAction(arm,armSlider, 1090, 1)
                        //new IntakeAction(RServo,LServo,0,1) // Reset servo power
                        //new PivotServoAction(pivotServo, 0.85) //0.7
                ),
                //arm 15 -> 249
                //slider 741 -> 2763
                //
                new SequentialAction(
                        wait1Second,
                        new ArmSliderAction(armSlider, 1875, 1),
                        new IntakeAction(RServo,LServo,-0.75,1.7),
                        new ArmSliderAction(armSlider, -1380,1),//1190
                        wait1Second
                ),

                //slider 400
                //arm 78

                new ParallelAction(
                        turnToBlock1,
                        new ArmAction(arm,armSlider, -1060, 0.4), //-1050
                        new PivotServoAction(pivotServo,0.611)//0.65

                ),

                new ParallelAction(
                        pickBlock1,
                        new IntakeAction(RServo, LServo, 1, 2.6)
                ),

                new SequentialAction(
                        moveToBasket2,
                        new ArmAction(arm, armSlider, 1324, 0.7),//1290
                        new ArmSliderAction(armSlider,1100,1),//1040
                        wait05Second,
                        new PivotServoAction(pivotServo, 0.46),
                        new IntakeAction(RServo,LServo,-0.47,1.7),//0.55
                        new PivotServoAction(pivotServo, 0.611),

                        new ArmSliderAction(armSlider,-940,1)//-700
                ),

                new ParallelAction(
                        turnToBlock2,
                        new ArmAction(arm,armSlider, -1360,0.4)
                        //new PivotServoAction(pivotServo,0.615)
                ),

                wait1Second,

                new ParallelAction(
                        pickBlock2,
                        new IntakeAction(RServo, LServo, 1, 2.7)

                ),
                new SequentialAction(
                        moveToBasket3,
                        new ArmAction(arm,armSlider, 1370, 0.7),
                        new ArmSliderAction(armSlider,875,1),//715
                        wait1Second,
                        new PivotServoAction(pivotServo, 0.52),
                        new IntakeAction(RServo,LServo,-0.48,1.6)
//                                new ArmSliderAction(armSlider,-700,1)
                ),
                new ParallelAction(
                        faceForward,
                        new ArmAction( arm, armSlider,-1300,0.7),
                        new ArmSliderAction(armSlider,-1300,1)
                )
//                                .strafeToLinearHeading(new Vector2d(-48,-40),Math.toRadians(90))
//                                .waitSeconds(2)
//                                .strafeToLinearHeading(new Vector2d(-52, -52),Math.toRadians(225))
//                                .waitSeconds(4)
//                                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
//                                .waitSeconds(3)
//                                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(225))
//                                .waitSeconds(4)
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

            if (armSlider.getCurrentPosition() > 200) {
                arm.setPower(0.35);
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

    public class IntakeAction implements Action {
        CRServoImplEx RServo;
        CRServoImplEx LServo;
        double intakePower;
        double intakeTime;
        ElapsedTime timer;

        public IntakeAction(CRServoImplEx RServo, CRServoImplEx LServo, double intakePower, double intakeTime) {
            this.RServo = RServo;
            this.LServo = LServo;
            this.intakePower = intakePower;
            this.intakeTime = intakeTime;
            timer = null;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                RServo.setPwmEnable();
                LServo.setPwmEnable();
                RServo.setPower(-intakePower);
                LServo.setPower(intakePower);
            }

            if (timer.seconds() <= intakeTime) {
                return true;
            } else {
                RServo.setPower(0);
                LServo.setPower(0);
                RServo.setPwmDisable();
                LServo.setPwmDisable();
                return false;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////
    public class PatientClawAction implements Action {
        Servo claw;
        double clawPos;
        ElapsedTime timer;

        public PatientClawAction(Servo claw, double clawPos) {

            this.claw = claw;
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                claw.setPosition(clawPos);

            }

            if (timer.seconds() < 3) {
                return true;
            } else {
                return false;
            }
        }
    }

    public class PivotServoAction implements Action {
        ServoImplEx pivotServo;
        double pivotServoPos;
        ElapsedTime timer;

        public PivotServoAction(ServoImplEx pivotServo, double pivotServoPos) {

            this.pivotServo = pivotServo;
            this.pivotServoPos = pivotServoPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                pivotServo.setPosition(pivotServoPos);

            }

            if (timer.seconds() < 1) {
                return true;
            } else {
                return false;
            }
        }
    }
}