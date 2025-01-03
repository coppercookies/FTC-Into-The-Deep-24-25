package org.firstinspires.ftc.teamcode.RRTest;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RRtest1225", group = "Autonomous")

public class RRTest1225 extends LinearOpMode {

    public class Arm {
        private DcMotor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "armMotor");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new ArmDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-39, -65, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225.00));

        TrajectoryActionBuilder traj2 = drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(50,50),Math.toRadians(360));



        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        arm.armUp(),
                        traj2.build(),
                        claw.openClaw(),
                        arm.armDown()
                )
        );

        /*
        //Go to basket with pre-load and drop sample
                .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                .waitSeconds(4)
                //Go to first sample and go straight to pick up
                .strafeToLinearHeading(new Vector2d(-50,-40),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-50,-30),Math.toRadians(90))
                //Go to basket with first sample
                .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                .waitSeconds(4)
                //Go to second sample and pick up
                .strafeToLinearHeading(new Vector2d(-60,-40),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-60,-30),Math.toRadians(90))
                //Go to basket with second sample
                .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                .waitSeconds(4)

                //Go to third sample and pick up
                .strafeToLinearHeading(new Vector2d(-60,-40),Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(-60,-30),Math.toRadians(135))
                //Go to basket with third sample
                .strafeToLinearHeading(new Vector2d(-50, -50),Math.toRadians(225))
                .waitSeconds(4)
                .build()
            */

    }

}
