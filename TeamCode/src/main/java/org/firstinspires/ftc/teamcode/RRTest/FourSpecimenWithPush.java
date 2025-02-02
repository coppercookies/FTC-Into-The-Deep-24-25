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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous (name = "Red Back 4 Spec Backward Push")
public class FourSpecimenWithPush extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(14.4, -61.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(59),
                new AngularVelConstraint(Math.toRadians(180))
        ));
        VelConstraint baseMoveToSub = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(76),
                new AngularVelConstraint(Math.toRadians(180))
        ));
        //it worked at 50
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-52, 52);




        DcMotorEx clawSlider;
        ServoImplEx claw;

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        clawSlider = hardwareMap.get(DcMotorEx.class, "clawSlider");
        clawSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Actions.runBlocking(
                new ClawAction(claw, 0.7)
        );

        Action moveToSub1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(10,-31.5))
                .build();

        Action moveToAndPushBlock1 = drive.actionBuilder(new Pose2d(10,-31.5,Math.toRadians(180)))
                //get away from sub
                .strafeToConstantHeading(new Vector2d(10,-33))
                //move in front of first block
                .splineToLinearHeading(new Pose2d(36,-30,Math.toRadians(90)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, -5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48.2, -5), Math.toRadians(270))
                //move back to observation zone
                .splineToConstantHeading(new Vector2d(48.2, -51), Math.toRadians(90))
                .build();

        Action moveToAndPushBlock2 = drive.actionBuilder(new Pose2d(48.2,-51,Math.toRadians(90)))
                //move in front of second block
                .splineToConstantHeading(new Vector2d(48.2, -5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, -5), Math.toRadians(-90))
                //move back to observation zone
                .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(270.00))
                .build();

        Action moveToPickup1 = drive.actionBuilder(new Pose2d(58,-57,Math.toRadians(90)))
                //move away from pushed block and go towards first specimen
                .strafeToLinearHeading(new Vector2d(45,-45),Math.toRadians(360))
                .strafeToConstantHeading(new Vector2d(31.3,-58.2))
                .build();

        Action moveToSub2 = drive.actionBuilder(new Pose2d(31.3,-58.2,Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(10, -28),Math.toRadians(180))
                .build();

        Action moveToPickup2 = drive.actionBuilder(new Pose2d(10, -28, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(26.5, -50),Math.toRadians(360))
                .strafeTo(new Vector2d(26.5,-58.8))
                .build();

        Action moveToSub3 = drive.actionBuilder(new Pose2d(26.5, -58.8, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(5, -28),Math.toRadians(180))
                .build();


        Action moveToPickup3 = drive.actionBuilder(new Pose2d(5,-28, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(27.5, -50),Math.toRadians(360),baseVelConstraint, baseAccelConstraint)
                .strafeTo(new Vector2d(27.5,-58.8), new TranslationalVelConstraint(8))
                .build();

        Action moveToSub4 = drive.actionBuilder(new Pose2d(27.5, -58.8, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(1, -27), Math.toRadians(180), baseMoveToSub, baseAccelConstraint)
                .build(); //3



        Action turn = drive.actionBuilder((new Pose2d(1,-28,Math.toRadians(180))))
                .strafeToLinearHeading(new Vector2d(3,-35),Math.toRadians(90))
                .build();





        waitForStart();

        Actions.runBlocking(new SequentialAction(

                        new ParallelAction(
                                moveToSub1,
                                new ClawSliderAction(clawSlider,-2100,1)
                        ),

                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                new PatientClawAction(claw, 0.0)
                        ),

                        new ParallelAction(
                                moveToAndPushBlock1,
                                new ClawSliderAction(clawSlider,1395,0.6)
                        ),

                        new SequentialAction(
                                moveToAndPushBlock2,
                                moveToPickup1,
                                new PatientClawAction(claw,0.7)
                        ),
                        new ParallelAction(
                                moveToSub2,
                                new ClawSliderAction(clawSlider,-1695,0.9)
                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                new PatientClawAction(claw, 0.0)
                        ),
                        //Third spec
                        new ParallelAction(
                                moveToPickup2,
                                new ClawSliderAction(clawSlider, 1395, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7)

                        ),
                        new ParallelAction(
                                moveToSub3,
                                new ClawSliderAction(clawSlider, -1695, 0.9)

                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300, 0.8),
                                new PatientClawAction(claw, 0.0)

                        ),
                        //Fourth spec
                        new ParallelAction(
                                moveToPickup3,
                                new ClawSliderAction(clawSlider, 1395, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7)

                        ),
                        new ParallelAction(
                                moveToSub4,
                                new ClawSliderAction(clawSlider, -1695, 0.9)

                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 300 , 0.8),
                                new PatientClawAction(claw, 0.0)

                        ),
                        //park
                        new ParallelAction(
                                turn,
                                new ClawSliderAction(clawSlider,1820,1)
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

            if (timer.seconds() < 0.1) {
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
                startClawSliderPos = clawSlider.getCurrentPosition();
                clawSlideEndPos = clawSliderPos + startClawSliderPos;
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