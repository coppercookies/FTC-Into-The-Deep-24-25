package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class DcMotorTest extends OpMode {

    /* Declare OpMode members. */
    private DcMotor clawSlider;

    public void init() {
        // Define and Initialize Motors
        clawSlider = hardwareMap.get(DcMotor.class, "clawSlider");



    }
    public void loop() {
        double clawSliderPower;

        if (gamepad1.x) {
            clawSliderPower = 1;
        } else if (gamepad1.b) {
            clawSliderPower = -1;
        } else {
            clawSliderPower = 0;
        }
        clawSlider.setPower(clawSliderPower);

    }
}
