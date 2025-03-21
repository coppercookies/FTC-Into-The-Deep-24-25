package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class FCDOutreach extends OpMode {

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

    private double armPower = 0.5;
    private double wristPower = 0;

    private boolean dcMotorEnabled = false;


    private boolean clawClosed;
    private boolean reverseArm;

    IMU imu;
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        clawClosed = false;
        reverseArm = false;

    }


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
            wristPower = -0.4;

        } else if (gamepad1.a) {
            //DOWN "a"
            telemetry.addData("a", true);
            telemetry.addData("CurrentWristPos", wrist.getCurrentPosition());
            int currentwristPos = wrist.getCurrentPosition();
            if (currentwristPos > zeroWristPos + 50)
                wristPower = 0;
            else
                wristPower = 0.5;
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

        arm.setPower(armPower);
        wrist.setPower(wristPower);
        FCDDrive();
    }

    private void FCDDrive() {
        double vertical = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double turn = -gamepad2.right_stick_x;

        //max power over here is 1.0
        double drivePower = 1.0 - (0.63 * gamepad2.right_trigger);

        if (gamepadRateLimit.hasExpired() && gamepad2.start) {
            imu.resetYaw();
            gamepadRateLimit.reset();
        }

//        if (gamepad2.a && AClicked) {
//            if (gamepadRateLimit.hasExpired() && gamepad2.a) {
//                imu.resetYaw();
//                gamepadRateLimit.reset();
//                AClicked = false;
//                //                max_LED_red.off();
//                //                max_LED_green.on();
//            }
//            telemetry.addData("AClicked", AClicked);
//        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double adjustedVertical = vertical * Math.cos(heading) - strafe * Math.sin(heading);
        double adjustedStrafe = vertical * Math.sin(heading) + strafe * Math.cos(heading);

        double max = Math.max(Math.abs(adjustedStrafe) + Math.abs(adjustedVertical) + Math.abs(turn), 1);

        double RFPower = ((turn + (adjustedVertical - adjustedStrafe)) / max) * drivePower;
        double RBPower = ((turn + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LFPower = (((-turn) + (adjustedVertical + adjustedStrafe)) / max) * drivePower;
        double LBPower = (((-turn) + (adjustedVertical - adjustedStrafe)) / max) * drivePower;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

        rightFront.setPower(RFPower);
        rightBack.setPower(RBPower);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
}

