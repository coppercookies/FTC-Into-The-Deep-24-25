package org.firstinspires.ftc.teamcode.finalCodes;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous (name = "4SpecTest3Dead")
public class newTest0201 extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(14.4, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

//        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
//            if (robotPose.position.x.value() > 24.0) {
//                return 20.0;
//            } else {
//                return 50.0;
//            }
//        }

        DcMotorEx clawSlider;
        ServoImplEx claw;

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        clawSlider = hardwareMap.get(DcMotorEx.class, "clawSlider");
        clawSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //first too low
        //
        Actions.runBlocking(
                new ClawAction(claw, 0.7)
        );

        Action moveToSub1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(15,-31.5),new TranslationalVelConstraint(80))
                .build();

        Action moveToBlockAndPush = drive.actionBuilder(new Pose2d(15,-31.5,Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(15,-34))

                .splineTo(new Vector2d(35, -24), Math.toRadians(90))//35
                .splineTo(new Vector2d(35, -10), Math.toRadians(90))
                //push block
                .splineToConstantHeading(new Vector2d(48.2, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48.2, -51), Math.toRadians(90))
                .build();

        Action pushBlock2 = drive.actionBuilder(new Pose2d(48.2,-51,Math.toRadians(360)))

                .strafeTo(new Vector2d(48.2, -8))

                .strafeTo(new Vector2d(59,-8))
                .strafeTo(new Vector2d(59,-57))

                .build();

        Action moveToPickup1 = drive.actionBuilder(new Pose2d(59,-57,Math.toRadians(360)))
                .strafeToConstantHeading(new Vector2d(27.99, -52))
                .strafeToConstantHeading(new Vector2d(27.99,-61.3))
                .build();

        Action moveToSub2 = drive.actionBuilder(new Pose2d(27.99,-61.3,Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(12, -31.5),Math.toRadians(180))
                .build(); //9

        Action moveToPickup2 = drive.actionBuilder(new Pose2d(12, -31.5, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(28, -55),Math.toRadians(360))
                .strafeTo(new Vector2d(27.99,-61.3))

                .build();

        Action moveToSub3 = drive.actionBuilder(new Pose2d(27.99, -61.3, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(8, -31.5),Math.toRadians(180))
                .build();//6


        Action moveToPickup3 = drive.actionBuilder(new Pose2d(8,-31.5,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(27.99, -55),Math.toRadians(360))
                .strafeTo(new Vector2d(27.99,-61.3))

                .build();

        Action moveToSub4 = drive.actionBuilder(new Pose2d(27.99, -61.3, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(4, -31.5), Math.toRadians(180))
                .build(); //3


        Action park = drive.actionBuilder(new Pose2d(4, -31.5, Math.toRadians(180)))
                .strafeTo(new Vector2d(2,-40))
                .strafeToLinearHeading(new Vector2d(47,-54),Math.toRadians(90), new TranslationalVelConstraint(100))
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
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.0)
                        ),

                        new ParallelAction(
                                moveToBlockAndPush,
                                new ClawSliderAction(clawSlider,1375,0.8)
                        ),
//
                        new SequentialAction(
                                pushBlock2,
                                moveToPickup1,
                                new PatientClawAction(claw,0.7),
                                wait05Seconds
                        ),
////425
                        new ParallelAction(
                                moveToSub2,
                                new ClawSliderAction(clawSlider,-1675,0.9)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.0)
                        ),
                        //Third spec
                        new ParallelAction(
                                moveToPickup2,
                                new ClawSliderAction(clawSlider, 1375, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7)
//
                        ),
                        new ParallelAction(
                                moveToSub3,
                                new ClawSliderAction(clawSlider, -1675, 0.9)

                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                wait05Seconds,
                                new PatientClawAction(claw, 0.0)

                        ),
                        //Fourth spec
                        new ParallelAction(
                                moveToPickup3,
                                new ClawSliderAction(clawSlider, 1375, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7),
                                wait05Seconds

                        ),
                        new ParallelAction(
                                moveToSub4,
                                new ClawSliderAction(clawSlider, -1685, 0.9)
//
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300 , 0.8),
                                new PatientClawAction(claw, 0.0)

                        ),

                        new ParallelAction(
                                park,
                                new ClawSliderAction(clawSlider,1800,1)
                        )
//


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