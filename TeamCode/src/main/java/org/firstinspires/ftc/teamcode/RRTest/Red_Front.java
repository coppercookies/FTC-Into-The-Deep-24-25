package org.firstinspires.ftc.teamcode.RRTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous (name = "RedFront")
public class Red_Front extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(-40, -62, Math.toRadians(180));
        Pose2d beginPose = new Pose2d(-38, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx arm;
        DcMotorEx armSlider;
        CRServoImplEx RServo;
        CRServoImplEx LServo;

        ServoImplEx claw;
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armSlider = hardwareMap.get(DcMotorEx.class, "armSlider");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        RServo = hardwareMap.get(CRServoImplEx.class,"RServo" );
        LServo = hardwareMap.get(CRServoImplEx.class,"LServo" );
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        Actions.runBlocking(
                //new ParallelAction(
                        drive.actionBuilder(beginPose)
                                //.strafeToLinearHeading(new Vector2d(-38 - 30, -61.5 +  7.5),Math.toRadians(180))
                                .strafeToLinearHeading(new Vector2d(-52,-26),Math.toRadians(180))
                                .waitSeconds(4)
//                                .strafeToLinearHeading(new Vector2d(-48,-40),Math.toRadians(90))
//                                .waitSeconds(2)
//                                .strafeToLinearHeading(new Vector2d(-52, -52),Math.toRadians(225))
//                                .waitSeconds(4)
//                                .strafeToLinearHeading(new Vector2d(-58,-40), Math.toRadians(90))
//                                .waitSeconds(3)
//                                .strafeToLinearHeading(new Vector2d(-52,-52),Math.toRadians(225))
//                                .waitSeconds(4)
                                .build()
            //)
        );


    }


    public class ArmAction implements Action {
        DcMotor arm;
        double armPower;
        int armPos;
        boolean initialized;
        int startArmPos;
        int armEndPos;
        boolean goingUp;
        public ArmAction(DcMotor arm, int armPos, double armPower) {
            this.arm = arm;
            this.armPower = armPower;
            this.armPos = armPos;
            this.initialized = false;
            this.startArmPos = 0;
            this.armEndPos = 0;
            if (armPos > 0)
                this.goingUp = false;
            else
                this.goingUp = true;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                startArmPos = arm.getCurrentPosition();
                armEndPos = armPos + startArmPos;
                arm.setTargetPosition(armEndPos);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                initialized = true;
            }

            if (goingUp) {
                if (arm.getCurrentPosition() >= armEndPos) {
                    return true;
                }
            }
            else {
                if (arm.getCurrentPosition() <= armEndPos) {
                    return true;
                }
            }
//            if (armSlider.getCurrentPosition() != armSliderPos) {
//                return true;
//            }
            arm.setPower(0.25);
            return false;
        }
    }


    public class ArmSliderAction implements Action {
        DcMotor armSlider;
        double armSliderPower;
        int armSliderPos;
        boolean initialized;
        int startArmSliderPos;
        int armSliderEndPos;
        boolean goingUp;
        public ArmSliderAction(DcMotor armSlider, int armSliderPos, double armSliderPower) {
            this.armSlider = armSlider;
            this.armSliderPower = armSliderPower;
            this.armSliderPos = armSliderPos;
            this.initialized = false;
            this.startArmSliderPos = 0;
            this.armSliderEndPos = 0;
            if (armSliderPos > 0)
                this.goingUp = false;
            else
                this.goingUp = true;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                startArmSliderPos = armSlider.getCurrentPosition();
                armSliderEndPos = armSliderPos + startArmSliderPos;
                armSlider.setTargetPosition(armSliderEndPos);
                armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlider.setPower(armSliderPower);
                initialized = true;
            }

            if (goingUp)
            {
                if (armSlider.getCurrentPosition() >= armSliderEndPos) {
                    return true;
                }
            }
            else
            {
                if (armSlider.getCurrentPosition() <= armSliderEndPos) {
                    return true;
                }
            }
//            if (armSlider.getCurrentPosition() != armSliderPos) {
//                return true;
//            }
            //Negative goes up
            armSlider.setPower(0);
            return false;
        }
    }


    public class IntakeAction implements Action {
        CRServo RServo;
        CRServo LServo;
        double intakePower;

        public IntakeAction(CRServo RServo,CRServo LServo, double intakePower) {
            this.RServo = RServo;
            this.LServo = LServo;
            this.intakePower = intakePower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            RServo.setPower(intakePower);
            LServo.setPower(-intakePower);
            return false;
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
            if(timer == null) {
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
}