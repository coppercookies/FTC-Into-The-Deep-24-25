package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

@Disabled

@TeleOp(name = "Try Slider LED and Touch", group = "Concept")
public class TrySliderLEDAndTouch extends OpMode {

    DigitalChannel digitalTouch;
    LED slider_LED_red;
    LED slider_LED_green;

    @Override
    public void init() {
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        slider_LED_green = hardwareMap.get(LED.class, "slider_led_green");
        slider_LED_red = hardwareMap.get(LED.class, "slider_led_red");

    }

    @Override
    public void loop() {
        if (digitalTouch.getState() == true) {
            telemetry.addData("Touch Sensor Pressed", true);
        } else {
            telemetry.addData("Touch Sensor Pressed", false);
        }


        if (digitalTouch.getState() == true) {
            slider_LED_green.on();
            slider_LED_red.off();
        } else {
            slider_LED_green.off();
            slider_LED_red.on();
        }



    }
}




