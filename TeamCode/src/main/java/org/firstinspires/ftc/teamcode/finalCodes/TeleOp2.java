//24-25 Copper Cookies TeleOp
package org.firstinspires.ftc.teamcode.finalCodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp2", group="Iterative OpMode")
public class TeleOp2 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // DcMotors --------------------------------------------------------
    // private DcMotor hanging;

    private DcMotor armSlider;
    private int maxArmSliderPos;
    private int zeroArmSliderPos;

    private DcMotor clawSlider;
    private int zeroClawSliderPos;
    private final int zeroClawSliderPosCorrection = 470;
    private int maxClawSliderPos;

    private DcMotor arm;

    private int maxPos;
    private int zeroArmPos;

    private boolean AClicked = true;
    private boolean YClicked = true;

    private DcMotor rightFront, leftFront, rightBack, leftBack;

    // Servos ----------------------------------------------------------
    private Servo claw;
    private boolean clawEnabled;

    private ServoImplEx armClaw;
    private CRServo pivotClaw;


    private Servo sweeper;    //Sensors + etc
    LED max_LED_red;
    LED max_LED_green;
    Servo LED_Headlight;

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

        pivotClaw = hardwareMap.get(CRServo.class, "pivotClaw");
        armClaw = hardwareMap.get(ServoImplEx.class, "armClaw");

        //        RServo = hardwareMap.get(CRServo.class, "RServo");
        //        LServo = hardwareMap.get(CRServo.class, "LServo");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        // Init IMU-- ------------------------------------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        // Init ClawSlider + Claw -----------------------------------------------
        clawSlider = hardwareMap.get(DcMotor.class, "clawSlider");
        zeroClawSliderPos = clawSlider.getCurrentPosition();
        zeroClawSliderPos = zeroClawSliderPos - zeroClawSliderPosCorrection;
        maxClawSliderPos = zeroClawSliderPos - 1500;//-2150
        //clawSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //clawSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        clawEnabled = false;

        // Init Arm -----------------------------------------------------------
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int zeroArmPosCorrection = 80;
        zeroArmPos = (int)ReadWrite.readFromFile("ZeroArmPos.txt") + zeroArmPosCorrection;
        telemetry.addData("zeroArmPosfromFile", zeroArmPos);

        // -1185 + 1000 = -185
        // Was 1030 on 3/21
        maxPos = zeroArmPos + 950; // + zeroArmPosCorrection; // Add number for max

        // Init ArmSlider -----------------------------------------------------
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        //armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //zeroArmSliderPos = armSlider.getCurrentPosition();
        zeroArmSliderPos = (int)ReadWrite.readFromFile("ZeroArmSliderPos.txt");
        //maxArmSliderPos = zeroArmSliderPos + 2525;//2000
        maxArmSliderPos = zeroArmSliderPos + 2700;//2525
        telemetry.addData("CurrentArmSliderPos", armSlider.getCurrentPosition());

        //Sensors + LED
        max_LED_green = hardwareMap.get(LED.class, "max_LED_green");
        max_LED_red = hardwareMap.get(LED.class, "max_LED_red");
        LED_Headlight = hardwareMap.get(Servo.class, "LED_Headlight");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        FCDDrive();
        ClawSlider();
        ArmClaw();
        PivotClaw();
        Arm();
        ArmSlider();
        Claw();
        Claw();
        Headlight();
        ZeroPos();
        PivotClaw();
        SweepServo();
    }

    private void Headlight() {
        if (gamepad2.y)
            LED_Headlight.setPosition(1);
        else if (gamepad2.a)
            LED_Headlight.setPosition(0);
    }
    private void SweepServo() {
        if (gamepad2.right_bumper) {
            sweeper.setPosition(0.19);
        } else{
            sweeper.setPosition(0.58);
        }

    }
    private void ZeroPos() {
        if (gamepad2.y && YClicked) {
            zeroArmSliderPos = armSlider.getCurrentPosition();
            //            zeroArmSliderPos = armSlider.getCurrentPosition();
            zeroClawSliderPos = clawSlider.getCurrentPosition() - zeroClawSliderPosCorrection;
            maxClawSliderPos = zeroClawSliderPos - 1500;
            //   zeroClawSliderPos = clawSlider.getCurrentPosition() + zeroClawSliderPosCorrection;
            YClicked = false;
        }
        telemetry.addData("YClicked", YClicked);
    }

    //Gamepad 1 Left Stick
    // Used to pick the sample
    private void ClawSlider() {
        double clawSliderPower;
        ///////////////////////////////// CLAW_SLIDER CODE
        telemetry.addData("gamepad1 y stick", gamepad1.right_stick_y);

        if (YClicked){
            if (gamepad1.right_stick_y > 0) {
                telemetry.addData("clawSlider down", true);
                clawSliderPower = 0.6;
            }
            else {
                clawSliderPower = 0.0;
            }
            clawSlider.setPower(clawSliderPower);
            return;
        }

        if (gamepad1.right_stick_y < 0 && (clawSlider.getCurrentPosition() >= maxClawSliderPos)) {
            telemetry.addData("clawSlider up", true);
            //            if (gamepad1.right_stick_y < 0.5) {
            //                clawSliderPower = -0.4;
            //            } else {
            //                clawSliderPower = -0.8;
            //            }
            clawSliderPower = -1;

        } else if (gamepad1.right_stick_y > 0 && (clawSlider.getCurrentPosition() <= zeroClawSliderPos)) {
            telemetry.addData("clawSlider down", true);
            clawSliderPower = 0.6;

            //        } else if (gamepad1.right_bumper) {
            //            //520
            //            if (clawSlider.getCurrentPosition() < (zeroClawSliderPos - 470)) {
            //                clawSliderPower = 0.5;
            //            } else if (clawSlider.getCurrentPosition() > (zeroClawSliderPos - 470)) {
            //                clawSliderPower = -0.5;
            //            }
        }
        else {
            clawSliderPower = -0.055;

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
            if (armSlider.getCurrentPosition() > zeroArmSliderPos) {
                //if (armSlider.getCurrentPosition() > zeroArmSliderPos) {
                armSliderPower = -1.0;
            }
            //}
        } else if (gamepad1.y) {
            //telemetry.addData("a", true);
            //telemetry.addData("Current armSliderPos", armSlider.getCurrentPosition());
            armSliderPower = 1.0;
            if (armSlider.getCurrentPosition() >= maxArmSliderPos) {
                armSliderPower = 0;
                LED_Headlight.setPosition(0.6);
            }
        }
        if (arm.getCurrentPosition() > 700) {
            maxArmSliderPos = zeroArmSliderPos + 2525;
        } else {
            maxArmSliderPos = zeroArmSliderPos + 2820;
            maxArmSliderPos = zeroArmSliderPos + 2000;//2820
        }
        //        double servoPosition = Math.max(0, Math.min(armSlider.getCurrentPosition() / 5000, 1));
        //        pivotServo.setPosition(servoPosition);
        if (armSlider.getCurrentPosition() >= maxArmSliderPos) {
            max_LED_green.on();
            max_LED_red.off();
        } else {
            max_LED_green.off();
            max_LED_red.on();
        }

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
            armPower = 0.025;
            if (arm.getCurrentPosition() <= zeroArmPos) {
                if (armSlider.getCurrentPosition() > 200) {
                    armPower = 0.75;
                } else {
                    armPower = 0.3;
                }
            } else if (arm.getCurrentPosition() + 500 >= maxPos) {
                armPower = -0.2;
            }
        } else if (gamepad1.dpad_up) {
            telemetry.addData("dpad_up", true);
            armPower = 0.75;
            if (arm.getCurrentPosition() >= maxPos) {
                armPower = 0;
            }
        } else {
            int currentArmPos = arm.getCurrentPosition();

            if (currentArmPos <= maxPos) {
                if (currentArmPos >= zeroArmPos + 600)
                    armPower = 0.14;//0.2
                else if (armSlider.getCurrentPosition() > zeroArmSliderPos + 200)
                    armPower = 0.27;//0.35
                else
                    armPower = 0.2;
            }
        }
        arm.setPower(armPower);

        telemetry.addData("Current Arm Position: ", arm.getCurrentPosition());
        telemetry.addData("Current Arm Power: ", armPower);
    }

    private void PivotClaw() {
        double pivotClawPower;

        if (gamepad1.dpad_right) {
            pivotClawPower = 0.35;
        } else if (gamepad1.dpad_left) {
            pivotClawPower = -0.35;
        } else {
            pivotClawPower = 0;
        }
        pivotClaw.setPower(pivotClawPower);
    }

    private void ArmClaw() {
        //        if (gamepad1.x) {
        //            armClawPower = -0.35;
        //
        //        }// Open claw
        //        else if(gamepad1.b) {
        //            armClawPower = 0.2;
        //        }
        //        else {
        //            armClawPower = 0;
        //        }
        //        telemetry.addData("arm claw power", armClawPower);
        //        armClaw.setPower(armClawPower);

        armClaw.scaleRange(0, 1);
        if (gamepad1.x) {
            armClaw.setPosition(0.65);//0.4
        } else if (gamepad1.b) {
            armClaw.setPosition(0.92);//0.695
        }
    }

    private void Claw() {
        if (!clawEnabled)
            return;
        //double ClawPower = 0.0;
        // if (gamepad1.right_bumper) {
        //   claw.setPosition(0.3); // close claw
        //} else
        if (gamepad1.left_bumper) {
            claw.setPosition(0.4); //0// Open claw
        } else {
            claw.setPosition(0.74);
        }
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
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));


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
        if (RFPower != 0) {
            clawEnabled = true;
        }

        rightFront.setPower(RFPower);
        rightBack.setPower(RBPower);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
    }
}