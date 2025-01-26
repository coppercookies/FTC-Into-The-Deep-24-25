package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled

@TeleOp(name = "Try LED and Touch", group = "Concept")
public class TryLEDAndTouch extends OpMode {

    RevTouchSensor touchSensor;
    LED intake_LED_red;
    LED intake_LED_green;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

        intake_LED_green = hardwareMap.get(LED.class, "intake_led_green");
        intake_LED_red = hardwareMap.get(LED.class, "intake_led_red");

    }

    @Override
    public void loop() {
//        if (touchSensor.isPressed() == true) {
//            telemetry.addData("Touch Sensor Pressed", true);
//        } else if (touchSensor.isPressed() == false) {
//            telemetry.addData("Touch Sensor Pressed", false);
//        }
//        telemetry.update();
//
//        if (gamepad1.a) {
//            intake_LED_green.on();
//            intake_LED_red.off();
//        } else if (gamepad1.y) {
//            intake_LED_green.off();
//            intake_LED_red.on();
//        } else {
//            intake_LED_green.off();
//            intake_LED_red.off();
//        }

        if (touchSensor.isPressed()) {
            intake_LED_green.on();
            intake_LED_red.off();
        } else if (!touchSensor.isPressed()) {
            intake_LED_green.off();
            intake_LED_red.on();
        }
        telemetry.addData("Touch Sensor Pressed", touchSensor.isPressed());





    }
}




