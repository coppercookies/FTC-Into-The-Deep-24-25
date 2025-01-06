//24-25 Copper Cookies TeleOp
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp2", group="Iterative OpMode")
public class TeleOp2 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // DcMotors --------------------------------------------------------
    private DcMotor hanging;
    private DcMotor armSlider;
    private int maxArmSliderPos;
    private int zeroArmSliderPos;
    private DcMotor clawSlider;
    private DcMotor arm;

    private int maxPos;
    private int zeroArmPos;
    private final int zeroArmPosCorrection = 180; //

    //    private boolean reverseArm;
    private CRServo LServo;
    private CRServo RServo;
    private Servo pivotServo;

    private double RServoPower;
    private double LServoPower;

    private DcMotor rightFront, leftFront, rightBack, leftBack;
    private double drift_motor_power = 1.0;

    // Servos ----------------------------------------------------------
    private CRServo intake;
    private Servo claw;

    IMU imu;
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);


    @Override
    public void init() {
        // Init Drive ---------------------------------------------------------
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        RServo = hardwareMap.get(CRServo.class, "RServo");
        LServo = hardwareMap.get(CRServo.class, "LServo");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        // Init IMU-- ------------------------------------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);


        // Init ClawSlider + Claw -----------------------------------------------
        clawSlider = hardwareMap.get(DcMotor.class, "clawSlider");

        //clawSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //clawSlider.setDirection(DcMotorSimple.Direction.REVERSE);


        claw = hardwareMap.get(Servo.class, "claw");

        // Init Hanging ------------------------------------------------------
        hanging = hardwareMap.get(DcMotor.class, "hanging");
        hanging.setDirection(DcMotor.Direction.REVERSE);
        //hanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init Arm -----------------------------------------------------------
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        zeroArmPos = arm.getCurrentPosition();
        telemetry.addData("CurrentArmPos", arm.getCurrentPosition());
        //zeroArmPos = zeroArmPos + zeroArmPosCorrection;
        telemetry.addData("zeroArmPos", zeroArmPos);
        maxPos = zeroArmPos + 1050; // Add number for max
        //it was - we cahnged to +
        //reverseArm = false;

        // Init ArmSlider -----------------------------------------------------
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        //armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeroArmSliderPos = armSlider.getCurrentPosition();
        maxArmSliderPos = zeroArmSliderPos + 2000;
        telemetry.addData("CurrentArmSliderPos", armSlider.getCurrentPosition());

        // Init Intake --------------------------------------------------------
        intake = hardwareMap.get(CRServo.class, "intake");

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Drive();
        FCDDrive();
        ClipDown();
        ClawSlider();
        Intake();
        Arm();
        ArmSlider();
        Hanging();
        Claw();
        PivotServo();
//        ClawSliderTest();
    }

    //Right Stick Button
    //ClawSlider goes up manually
    private void ClipDown() {
        if (gamepad1.right_stick_button) {
            //Clip specimen
            claw.setPosition(0.3);
            //Strafe towards bar and Align
            DriveToPos(350, -350, -350, 350, 0.3);
            //Bring clawSlider down and unclip Claw
            SetClawSliderPos(-300, 0.4);
            // Hold position
            clawSlider.setPower(-0.1);
            claw.setPosition(-0.3);
            // Let it drop
            clawSlider.setPower(0);
            //SetClawSliderPos(-500,0.4);
        }
    }

    private void Intake() {
//        double intakePower = 0.0;
//        /////////////////////////////////// INTAKE CODE
//        if (gamepad1.b) {
//            if (arm.getCurrentPosition() >= zeroArmPos)
//                arm.setTargetPosition(zeroArmPos + zeroArmPosCorrection);
//            telemetry.addData("intake in", true);
//            intakePower = -1.0;
//        } else if (gamepad1.x) {
//            if (arm.getCurrentPosition() >= zeroArmPos)
//                arm.setTargetPosition(zeroArmPos + zeroArmPosCorrection);
//            telemetry.addData("intake out", true);
//            intakePower = 1.0;
//        }
//        intake.setPower(intakePower);
//    }
        if (gamepad1.b) {

            telemetry.addData("intake in", true);
            RServoPower  = -1.0;
            LServoPower  = 1.0;
        } else if (gamepad1.x) {

            telemetry.addData("intake out", true);
            RServoPower = 1.0;
            LServoPower = -1.0;
        } else {
            RServoPower = 0;
            LServoPower = 0;
        }
        LServo.setPower(LServoPower);
        RServo.setPower(RServoPower);
    }

    private void PivotServo() {
        double pivotServoPos = pivotServo.getPosition();
        pivotServo.scaleRange(0,0.7);

        if (arm.getCurrentPosition() > zeroArmPos + 600) {
            pivotServo.setPosition(0.7);
        } else {
            if (gamepad1.dpad_right) {
                pivotServo.setPosition(0.1);
                pivotServoPos = pivotServo.getPosition();
            } else if (gamepad1.dpad_left) {
                pivotServo.setPosition(0.18); //0.2
                //parallel
            }
        }

        telemetry.addData("PivotServoPos", pivotServoPos);
    }


//    private void ClawSliderTest() {
//        /////////////////////////////////// ARM SLIDER CODE
//        double clawSliderTestPower = 0.0;
//        if (gamepad2.y) {
//            //UP "y"
//            telemetry.addData("y", true);
//            telemetry.addData("Current testClawSlider", clawSlider.getCurrentPosition());
//            clawSliderTestPower = -1.0;
//        } else if (gamepad2.a) {
//            //DOWN "a"
//            telemetry.addData("a", true);
//            telemetry.addData("Current testClawSlider", clawSlider.getCurrentPosition());
//            clawSliderTestPower = 1.0;
//        }
//        clawSlider.setPower(clawSliderTestPower);
//    }


    private void SetClawSliderPos(int clawSliderPos, double power) {
        int currentArmPos = clawSlider.getCurrentPosition();
        clawSlider.setTargetPosition(currentArmPos - clawSliderPos);
        clawSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawSlider.setPower(power);

        while (clawSlider.isBusy()) {
            Thread.yield();
        }
    }

    //Gamepad 1 Left Stick
    // Used to pick the sample
    private void ClawSlider() {
        double clawSliderPower = 0.0;
        ///////////////////////////////// CLAW_SLIDER CODE
        telemetry.addData("gamepad1 y stick", gamepad1.right_stick_y);
        if (gamepad1.right_stick_y < 0) {
            telemetry.addData("clawSlider up", true);
//            if (gamepad1.right_stick_y < 0.5) {
//                clawSliderPower = -0.4;
//            } else {
//                clawSliderPower = -0.8;
//            }
            clawSliderPower = -1;

        } else if (gamepad1.right_stick_y > 0) {
            telemetry.addData("clawSlider down", true);
            clawSliderPower = 0.6;
        } else {
            clawSliderPower = -0.1 ;
        }
        clawSlider.setPower(clawSliderPower);
        telemetry.addData("currentClawSliderPos", clawSlider.getCurrentPosition());
        //double clawSliderPower = gamepad2.right_stick_y;
        //clawSlider.setPower(clawSliderPower);
    }

    private void ArmSlider() {
        /////////////////////////////////// ARM SLIDER CODE
        double armSliderPower = 0.0;
        telemetry.addData("Current armSliderPos", armSlider.getCurrentPosition());
        if (gamepad1.a) {
            //UP "y"
            //telemetry.addData("y", true);
            //telemetry.addData("Current armSliderPos", armSlider.getCurrentPosition());
            armSliderPower = -1.0;
        } else if (gamepad1.y) {
            //DOWN "a"
            //telemetry.addData("a", true);
            //telemetry.addData("Current armSliderPos", armSlider.getCurrentPosition());
            armSliderPower = 1.0;
            if (armSlider.getCurrentPosition() >= maxArmSliderPos) {
                armSliderPower = 0;
            }
        }
//        double servoPosition = Math.max(0, Math.min(armSlider.getCurrentPosition() / 5000, 1));
//        pivotServo.setPosition(servoPosition);
        armSlider.setPower(armSliderPower);
    }


    private void Arm() {
        /////////////////////////////////// ARM CODE
/*        if (gamepad2.a)
        reverseArm = true;
    else if (gamepad2.y)
        reverseArm = false;
*/
        double armPower = 0.0;//0.0;

        if (gamepad1.dpad_down) {
            telemetry.addData("dPad_down", true);
            armPower = -0.3;
//            if (arm.getCurrentPosition() >= maxPos) {
//                armPower = 0;
//            }
            if (arm.getCurrentPosition() <= zeroArmPos ) {
                armPower = 0;
            }
        } else if (gamepad1.dpad_up) {
            telemetry.addData("dpad_up", true);
            armPower = 0.75;
            if (arm.getCurrentPosition() >= maxPos) {
                armPower = 0;
            }
//            if (arm.getCurrentPosition() <= zeroArmPos ) {
//            armPower = 0;
//            }
        } else {
            if (arm.getCurrentPosition() <= maxPos) {
                armPower = 0.2;
            }
        }
        arm.setPower(armPower);

        telemetry.addData("Current Arm Position: ", arm.getCurrentPosition());
        telemetry.addData("Current Arm Power: ", armPower);
    }

    private void Hanging() {
        double hangingPower = 0.0;
        if (gamepad1.left_stick_y > 0) {
            hangingPower = 0.9;
        } else if (gamepad1.left_stick_y < 0) {
            hangingPower = -0.9;
        }
        hanging.setPower(hangingPower);

    }

    private void Claw() {
        //double ClawPower = 0.0;
       // if (gamepad1.right_bumper) {
         //   claw.setPosition(0.3); // close claw
        //} else
//        if (gamepad1.left_bumper) {
//            claw.setPosition(0.6); // Open claw
//        } else {
//            claw.setPosition(0); // close claw
//        }
        if (gamepad1.left_bumper) {
            claw.setPosition(0); // Open claw
        } else {
            claw.setPosition(0.7); // close claw
        }

        //claw.setPosition(ClawPower);
    }

    private void DriveToPos(int leftFrontTarget,
                            int rightFrontTarget,
                            int leftBackTarget,
                            int rightBackTarget,
                            double speed) {
        int rightFrontPos;
        int rightBackPos;
        int leftFrontPos;
        int leftBackPos;

        rightFrontPos = rightFront.getCurrentPosition();
        leftFrontPos = leftFront.getCurrentPosition();
        leftBackPos = leftBack.getCurrentPosition();
        rightBackPos = rightBack.getCurrentPosition();

        rightFrontPos += rightFrontTarget;
        leftFrontPos += leftFrontTarget;
        rightBackPos += rightBackTarget;
        leftBackPos += leftBackTarget;

        rightFront.setTargetPosition(rightFrontPos);
        leftFront.setTargetPosition(leftFrontPos);
        rightBack.setTargetPosition(rightBackPos);
        leftBack.setTargetPosition(leftBackPos);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);

        while (rightFront.isBusy() && leftFront.isBusy()
                && rightBack.isBusy() && leftBack.isBusy()) {
            Thread.yield();
        }
    }

    private void Drive() {
        /////////////////////////////////// DRIVE CODE
        double leftFrontPower = 0.0;
        double rightFrontPower = 0.0;
        double leftBackPower = 0.0;
        double rightBackPower = 0.0;

        if (gamepad2.x) {
            drift_motor_power = 0.3;
        } else if (gamepad2.b) {
            drift_motor_power = 1.0;
        }
        if (gamepad2.right_stick_x > 0) {
            telemetry.addData("RS Turn Right", true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = -drift_motor_power;
            leftBackPower = drift_motor_power;
            rightBackPower = -drift_motor_power;
        } else if (gamepad2.right_stick_x < 0) {
            telemetry.addData("RS Turn Left", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = drift_motor_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.left_stick_y < 0) {
            telemetry.addData("LS Forward", true);
            telemetry.addData(String.valueOf(gamepad2.left_stick_y), true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = drift_motor_power;
            leftBackPower = drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.left_stick_y > 0) {
            telemetry.addData("LS Backward", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = -drift_motor_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = -drift_motor_power;
        } else if (gamepad2.dpad_right) {
            //telemetry.addData("Strafe Right", true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = -drift_motor_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.dpad_left) {
            //telemetry.addData("Strafe Left", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = drift_motor_power;
            leftBackPower = drift_motor_power;
            rightBackPower = -drift_motor_power;
        }
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    private void FCDDrive() {
        double vertical = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double turn = -gamepad2.right_stick_x;

        //max power oover here is 0.95
        double drivePower = 0.95 - (0.6 * gamepad2.right_trigger);

        if (gamepadRateLimit.hasExpired() && gamepad2.a) {
            imu.resetYaw();
            gamepadRateLimit.reset();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double adjustedVertical = vertical * Math.cos(heading) - strafe * Math.sin(heading);
        double adjustedStrafe = vertical * Math.sin(heading) + strafe * Math.cos(heading);



        double max = Math.max(Math.abs(adjustedStrafe) + Math.abs(adjustedVertical) + Math.abs(turn), 1);

        double RFPower = ((turn + (adjustedVertical - adjustedStrafe)) / max) * drivePower;
        double RBPower = ((turn + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LFPower = (((-turn) + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LBPower = (((-turn) + (adjustedVertical - adjustedStrafe)) / max) * drivePower;

        rightFront.setPower(RFPower);
        rightBack.setPower(RBPower);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
    }

}
/*
    private void Wrist() {
        ///////////////////////////////// WRIST CODE
        if (gamepad1.dpad_left) {
            telemetry.addData("wrist open", true);
            wristPower = 0.15;
        } else if (gamepad1.dpad_right) {
            telemetry.addData("wrist close", true);
            wristPower = -0.15;
        } else {
            telemetry.addData("wrist close", true);
            wristPower = 0;
        }
        wrist.setPower(wristPower);
    }
    private void Claw() {
        if (gamepad1.right_bumper) {
            Claw.setPosition(1.0);

        } else if (gamepad1.left_bumper) {
            Claw.setPosition(-1.0);
        }
    }
    private void SetArmPos(int armPos, double power){
        arm.setTargetPosition(zeroArmPos - armPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);

        while (arm.isBusy()) {
            Thread.yield();
        }
    }
*/
//417
//418
//419