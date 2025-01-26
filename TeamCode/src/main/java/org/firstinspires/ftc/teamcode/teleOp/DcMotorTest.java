package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled

@TeleOp
public class DcMotorTest extends OpMode {

    /* Declare OpMode members. */
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor arm;
    private DcMotor wrist;
    private CRServo claw; // port 1
    private Servo launchServo;
    private DcMotor launchDC;
    private int zeroArmPos;
    private int zeroWristPos;
    private int maxPos;

    private double drift_motor_power = 0.6;
    private double armPower = 0.5;
    private double wristPower = 0;
    private double top_right_add_power = 0;

    private double launchDCPower = 0;
    private boolean dcMotorEnabled = false;
    private double launchServoPower = 0.1;
    private double drone_enable_power = 0;

    private boolean clawClosed;
    private boolean reverseArm;
    private double rightFrontPower = 0;
    private double leftFrontPower = 0;
    private double rightBackPower = 0;
    private double leftBackPower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(CRServo.class, "claw"); // port 1
        launchDC = hardwareMap.get(DcMotor.class, "launchDC"); // Port 2 Control Hub
        launchServo = hardwareMap.get(Servo.class, "launchServo"); // Port 3 Control Hub
        launchServo.setPosition(0.3);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeroArmPos = arm.getCurrentPosition();
        zeroWristPos = wrist.getCurrentPosition();
        telemetry.addData("CurrentWristPos", wrist.getCurrentPosition());
        telemetry.addData("CurrentArmPos", arm.getCurrentPosition());
        telemetry.addData("zeroArmPos", zeroArmPos);
        maxPos = zeroArmPos - 4600; // Add number for max

        clawClosed = false;
        reverseArm = false;

    }


//    private void Set
//
//  Pos(int armPos, double power){
//        arm.setTargetPosition(zeroArmPos - armPos);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(power);
//
//        while (arm.isBusy()) {
//            Thread.yield();}
//    }
//
//    private void SetWristPos(int wristPos, double power){
//        wrist.setTargetPosition(zeroWristPos - wristPos);
//        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wrist.setPower(power);
//
//        while (wrist.isBusy()) {
//            Thread.yield();
//        }
//    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // claw code x for close b for open
        if (gamepad1.x) {
            telemetry.addData("Claw open", true);
            claw.setPower(0.3);
            clawClosed = false;
        } else if (gamepad1.b) {
            telemetry.addData("Claw close", true);
            claw.setPower(-0.3);
            clawClosed = true;
        } else {
            if (clawClosed)
                claw.setPower(-0.1);
            else
                claw.setPower(0);
        }

        // Drone Code ///////////////////////////////////
        if (gamepad1.right_bumper) {
            dcMotorEnabled = true;
        } else if (gamepad1.left_bumper) {
            dcMotorEnabled = false;
        }

        if (dcMotorEnabled)
        {
            //before the value was 0.656 (9/22)
            launchDC.setPower(1);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launchServo.setPosition(0.2);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launchDC.setPower(0);
            dcMotorEnabled = false;
        }
        else
        {
            launchDC.setPower(0);
            launchServo.setPosition(0.3);
        }

///////////////////////////////

        //wrist code--------------------------------------------
        if (gamepad1.y) {
//            UP "y"
            telemetry.addData("y", true);
            telemetry.addData("CurrentWristPos", wrist.getCurrentPosition());
            wristPower = -1;

        } else if (gamepad1.a) {
            //DOWN "a"
            telemetry.addData("a", true);
            telemetry.addData("CurrentWristPos", wrist.getCurrentPosition());
            int currentwristPos = wrist.getCurrentPosition();
            if (currentwristPos > zeroWristPos + 50)
                wristPower = 0;
            else
                wristPower = 1;
        }
        //IDLE
        else if (!gamepad1.y && !gamepad1.a) {
            telemetry.addData("wrist idle", true);
            wristPower = 0;
        }

        // ARM CODE-------------------------------------------------
        if (gamepad2.a)
            reverseArm = true;
        else if (gamepad2.y)
            reverseArm = false;

        if (gamepad1.dpad_up) {
            telemetry.addData("dPad_up", true);
            armPower = -0.6;
            if (arm.getCurrentPosition() < maxPos) {
                armPower = 0;
            }
        } else if (gamepad1.dpad_down) {
            telemetry.addData("dPad_down", true);
            armPower = 0.4;

            if (arm.getCurrentPosition() > zeroArmPos - 203) {
                armPower = 0;
            }
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            // If the arm is too heavy, this will prevent the arm from going down by putting little power up.
            telemetry.addData("dPad idle", true);
            if (reverseArm)
                armPower = 0.07;
            else
                armPower = -0.04;
        }
        telemetry.addData("Current Position: ", arm.getCurrentPosition());
        telemetry.addData("Current Power: ", armPower);

        // arm.setPower(armPower);
        telemetry.addData("zeroArmPos-400", zeroArmPos - 403);

//
//        //Auto Move wrist forward and pick up pixed.
//        if (gamepad1.left_bumper) {
//            arm.setTargetPosition(zeroArmPos);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setPower(0.2);
//
//            wrist.setTargetPosition(zeroWristPos);
//            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            wrist.setPower(0.2);
//            //SetWristPos(-zeroWristPos+100, 0.1);
//            telemetry.addData("Claw close", true);
//        }

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        if (gamepad2.x) {
            drift_motor_power = 0.3;
            top_right_add_power = 0;
        } else if (gamepad2.b) {
            drift_motor_power = 0.6;
            top_right_add_power = 0.0;
        }
        if (gamepad2.right_stick_x > 0) {
            telemetry.addData("RS Turn Right", true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = -drift_motor_power - top_right_add_power;
            leftBackPower = drift_motor_power;
            rightBackPower = -drift_motor_power;
        } else if (gamepad2.right_stick_x < 0) {
            telemetry.addData("RS Turn Left", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = drift_motor_power + top_right_add_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.left_stick_y < 0) {
            telemetry.addData("LS Forward", true);
            telemetry.addData(String.valueOf(gamepad1.left_stick_y), true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = drift_motor_power + top_right_add_power;
            leftBackPower = drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.left_stick_y > 0) {
            telemetry.addData("LS Backward", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = -drift_motor_power - top_right_add_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = -drift_motor_power;
        } else if (gamepad2.dpad_right) {
            //telemetry.addData("Strafe Right", true);
            leftFrontPower = drift_motor_power;
            rightFrontPower = -drift_motor_power - top_right_add_power;
            leftBackPower = -drift_motor_power;
            rightBackPower = drift_motor_power;
        } else if (gamepad2.dpad_left) {
//            telemetry.addData("Strafe Left", true);
            leftFrontPower = -drift_motor_power;
            rightFrontPower = drift_motor_power + top_right_add_power;

            leftBackPower = drift_motor_power;
            rightBackPower = -drift_motor_power;
        } else {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        arm.setPower(armPower);
        wrist.setPower(wristPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
}

