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

@Autonomous (name = "newTest0102")
public class newTest0102 extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(14.4, -61.5, Math.toRadians(180));
       // Pose2d beginPose = new Pose2d(9, -61.5, Math.toRadians(180));
//the begin pose above was for redback in the 3rd tournament
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.toRadians(180))
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-50, 50);



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

        Actions.runBlocking(
                new ClawAction(claw, 0.7)
        );

        Action moveToSub1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(15,-31.5), baseVelConstraint, baseAccelConstraint)
                .build();

        Action moveToBlock = drive.actionBuilder(new Pose2d(15,-31.5,Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(34,-42),Math.toRadians(360), baseVelConstraint, baseAccelConstraint)

                //Be above the block
                .strafeToConstantHeading(new Vector2d(36,-5), baseVelConstraint, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(46,-5), baseVelConstraint, baseAccelConstraint)
                .build();

        Action push2Blocks = drive.actionBuilder(new Pose2d(46,-5,Math.toRadians(360)))
                .strafeToConstantHeading(new Vector2d(46,-52), baseVelConstraint, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(46,-5) , baseVelConstraint, baseAccelConstraint)


                .strafeToConstantHeading(new Vector2d(55,-5),null, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(55,-52), null, baseAccelConstraint)
                .build();

        Action moveToPickup1 = drive.actionBuilder(new Pose2d(55,-52,Math.toRadians(360)))
                .strafeToConstantHeading(new Vector2d(55, -48), null, baseAccelConstraint)
                .strafeToConstantHeading(new Vector2d(26.5,-59.6), null, baseAccelConstraint)
                .build();


        Action moveToSub2 = drive.actionBuilder(new Pose2d(26.5,-59.6,Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(11, -28),Math.toRadians(180),baseVelConstraint, baseAccelConstraint)
                .build();

        Action moveToPickup2 = drive.actionBuilder(new Pose2d(11, -28, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(26.5, -54),Math.toRadians(360),baseVelConstraint, baseAccelConstraint)
                .strafeTo(new Vector2d(27,-59))
                .build();

        Action moveToSub3 = drive.actionBuilder(new Pose2d(26.5, -59, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(8, -28),Math.toRadians(180),baseVelConstraint, baseAccelConstraint)
                .build();


        Action moveToPickup3 = drive.actionBuilder(new Pose2d(8, -28 , Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(26.5, -54),Math.toRadians(360),baseVelConstraint, baseAccelConstraint)
                .strafeTo(new Vector2d(26.5,-59))
                .build();

        Action moveToSub4 = drive.actionBuilder(new Pose2d(26.5, -59, Math.toRadians(360)))
                .strafeToLinearHeading(new Vector2d(5, -28), Math.toRadians(180),baseVelConstraint, baseAccelConstraint)
                .build();


        Action park = drive.actionBuilder(new Pose2d(5, -28, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(47,-56),Math.toRadians(90), new TranslationalVelConstraint(100))
                .build();





        waitForStart();

        Actions.runBlocking(new SequentialAction(

                        new ParallelAction(
                                moveToSub1,
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
                                push2Blocks,
                                moveToPickup1,
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
                        //Third spec
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
                        //Fourth spec
                        new ParallelAction(
                                moveToPickup3,
                                new ClawSliderAction(clawSlider, 1395, 0.8)
                        ),
                        new SequentialAction(
                                new PatientClawAction(claw,0.7),
                                new ClawSliderAction(clawSlider, -400, 0.8)
                        ),
                        new ParallelAction(
                                moveToSub4,
                                new ClawSliderAction(clawSlider, -1300, 0.8)

                        ),
                        new SequentialAction(
                                new ClawSliderAction(clawSlider, 315, 0.8),
                                new PatientClawAction(claw, 0.0)

                        ),
                        //park
                        new ParallelAction(
                                park,
                                new ClawSliderAction(clawSlider,1780,0.8 )
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

            if (timer.seconds() < 0.3) {
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