//24-25 Copper Cookies TeleOp
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ChassisTeleOp", group="Iterative OpMode")
public class ChassisTeleOp extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();


    private DcMotor rightFront, leftFront, rightBack, leftBack;
    private double drift_motor_power = 1.0;



    @Override
    public void init() {
        // Init Drive ---------------------------------------------------------
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        NewDrive();
    }

    //Right Stick Button
    //Basket goes up manually



    private void NewDrive() {
        double vertical = 0;
        double strafe = 0;
        double turn = 0;

        vertical = gamepad2.left_stick_x;
        strafe = gamepad2.left_stick_y;
        turn = gamepad2.right_stick_x;

        rightFront.setPower(turn + (vertical + strafe));
        rightBack.setPower(turn + (vertical - strafe));
        leftFront.setPower((-turn) + (vertical - strafe));
        leftBack.setPower((-turn) + (vertical + strafe));
        /*
        Original was this (As per video)
        rightFront.setPower(turn + (-vertical + strafe));
        rightBack.setPower(turn + (-vertical - strafe));
        leftFront.setPower(turn + (-vertical - strafe));
        leftBack.setPower(turn + (-vertical + strafe));
         */
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