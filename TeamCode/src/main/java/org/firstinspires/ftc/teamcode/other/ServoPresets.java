package org.firstinspires.ftc.teamcode.other;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name = "Servo Presets")
public class ServoPresets extends LinearOpMode {


    public static double servo_pos = 0.5;


    @Override
    public void runOpMode() {
        Servo armClaw = hardwareMap.get(Servo.class,"armClaw");


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        telemetry.addData("Use FTC Dashboard to adjust the servo positions","");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {


            armClaw.setPosition(servo_pos);


            telemetry.addData("L Servo Pos: ", servo_pos);
            telemetry.update();
        }


    }
}

