//24-25 Copper Cookies TeleOp
package org.firstinspires.ftc.teamcode.teleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="clawTestZero", group="clawTest")
public class clawTestZero extends OpMode {
    // Declare OpMode members.

    private Servo armClaw;


    @Override
    public void init() {
        armClaw = hardwareMap.get(Servo.class, "armClaw");

    }


    @Override
    public void loop() {
        Claw();

    }

    private void Claw() {
/////////CLAW 2 - Back Up Claw Values

        if (gamepad1.x) {
            armClaw.setPosition(0.2); // Open claw
        } else if (gamepad1.b) {
            armClaw.setPosition(0.54); // Close claw
        }

    }
}