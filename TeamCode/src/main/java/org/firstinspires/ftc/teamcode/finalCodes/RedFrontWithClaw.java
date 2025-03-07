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
        DcMotorEx clawSlider;
        DcMotorEx armSlider;

        ServoImplEx claw;
        Servo armClaw;
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armSlider = hardwareMap.get(DcMotorEx.class, "armSlider");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        clawSlider = hardwareMap.get(DcMotorEx.class, "clawSlider");
        clawSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw = hardwareMap.get(ServoImplEx.class, "claw");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int zeroArmSliderPos = armSlider.getCurrentPosition();


        Action wait1Second = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .build();

        Action wait15Second = drive.actionBuilder(beginPose)
                .waitSeconds(1.5)
                .build();

        Action moveToBasket = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-38,-49))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(225))
                .build();
        Action moveForwardBasket = drive.actionBuilder(new Pose2d(-52.5,-52.5,Math.toRadians(225)))
                .strafeTo(new Vector2d(-55,-55))
                .build();
        Action moveBackBasket = drive.actionBuilder(new Pose2d(-55,-55,Math.toRadians(225)))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-48,-48))
                .build();


        Action turnToBlock1 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(225)))
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-45.5, -40.3), Math.toRadians(92))
                .build();


        Action moveToBasket2 = drive.actionBuilder(new Pose2d(-45.5,-40.3,Math.toRadians(92)))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(225))
                .build();

        Action moveForwardBasket2 = drive.actionBuilder(new Pose2d(-52.5,-52.5,Math.toRadians(225)))
                .strafeTo(new Vector2d(-55,-55))
                .build();
        Action moveBackBasket2 = drive.actionBuilder(new Pose2d(-55,-55,Math.toRadians(225)))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-50,-50))
                .build();

        Action turnToBlock2 = drive.actionBuilder(new Pose2d(-50, -50, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-54, -43), Math.toRadians(90))//-55.6
                .build();


        Action moveToBasket3 = drive.actionBuilder(new Pose2d(-54,-43,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(225))
                .build();

        Action moveForwardBasket3 = drive.actionBuilder(new Pose2d(-52.5,-52.5,Math.toRadians(225)))
                .strafeTo(new Vector2d(-55,-55))
                .build();
        Action moveBackBasket3 = drive.actionBuilder(new Pose2d(-55,-55,Math.toRadians(225)))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-50,-50))
                .build();

        Action park = drive.actionBuilder(new Pose2d(-50,-50,Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-30,-9),Math.toRadians(90), new TranslationalVelConstraint(120))
                .strafeToLinearHeading(new Vector2d(-16,-9),Math.toRadians(90), new TranslationalVelConstraint(90))

                .build();

        Actions.runBlocking(
                new SequentialAction(
                    new ArmAction(arm, armSlider, 75,0.2),
                    wait1Second,
                    new ArmClawAction(armClaw,0.73)

                )
        );
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                moveToBasket,
                                new ArmAction(arm,armSlider, 940, 1)

                        ),
                        new SequentialAction(
                               new ArmSliderAction(armSlider,2300,1),
                               moveForwardBasket,
                               new ArmAction(arm, armSlider,-100,1),
                               wait1Second,
                               new ArmClawAction(armClaw,0),
                               new ArmAction(arm, armSlider,140,1),
                               moveBackBasket,

                               new ArmSliderAction(armSlider, -2200,1),
                               wait1Second

                        ),
                        new SequentialAction(
                            new ArmAction(arm, armSlider, -975,0.05),
                            turnToBlock1,
                            wait1Second,
                            new ArmSliderAction(armSlider,690,1),
                            new ArmClawAction(armClaw,0.73),
                            new ArmSliderAction(armSlider,-590,1)
                        ),
                        new ParallelAction(
                            moveToBasket2,
                            new ArmAction(arm, armSlider,935,1)

                        ),
                        //go to basket with 2nd block
                        new SequentialAction(
                            new ArmSliderAction(armSlider,2370,1),

                            moveForwardBasket2,
                            wait1Second,
//                            new ArmAction(arm, armSlider,-100,1),
//                            wait1Second,
                            new ArmClawAction(armClaw,0),
                            new ArmAction(arm, armSlider,40,1),
                            moveBackBasket2,

                            new ArmSliderAction(armSlider, -2425,1)
                        ),


                new SequentialAction(

                        new ArmAction(arm, armSlider, -938,0.05),
                        turnToBlock2,
                        wait1Second,
                        new ArmSliderAction( armSlider, 935, 1),//875
                        wait1Second,
                        new ArmClawAction(armClaw,0.73),
                        new ArmSliderAction(armSlider,-765,1)
                ),
                new ParallelAction(
                 moveToBasket3,
                 new ArmAction(arm, armSlider,867,1)

                 ),
                new SequentialAction(
                        new ArmSliderAction(armSlider,2175,1),

                        moveForwardBasket3,
                        new ArmAction(arm, armSlider,-50,1),
                        wait1Second,
                        new ArmClawAction(armClaw,0),
                        wait1Second,
                        new ArmAction(arm, armSlider,90,1),
                        moveBackBasket3
    //                    new ArmSliderAction(armSlider, -2400,1),
    //                    wait1Second,
    //                    new ArmAction (arm, armSlider, -1050, 0.3),
    //                    park
                ),
                new ParallelAction(
                        new SequentialAction(
                                new ArmSliderAction(armSlider,-2400,1),
                                new ArmAction (arm, armSlider, -1020, 0.3)
                        ),
                        new ClawSliderAction(clawSlider, -1700,1),
                        new ClawAction(claw,0.73),
                        park

                )
//                new SequentialAction(
//                        new ClawSliderAction(clawSlider, 200,1)
//                )




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

            if (timer.seconds() < 1) {
                return true;
            } else {
                return false;
            }
        }
    }
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

}