package org.firstinspires.ftc.teamcode.RRTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "RRTeleOp", group = "TeleOp")
public class tryRRinTeleOp extends OpMode {

    private MecanumDrive drive;

    @Override
    public void init() {
        //Fix beginPose
        Pose2d beginPose = new Pose2d(9, -61.5, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        drive.localizer.setPose(beginPose);

    }

    @Override
    public void loop() {
        RRDrive();
    }

    private void RRDrive() {
        Pose2d pose = drive.localizer.getPose();
        double heading = pose.heading.toDouble(); // Get the robot's heading in radians

        double vertical = -gamepad2.left_stick_y;
        double strafe = -gamepad2.left_stick_x;

        double adjustedVertical = vertical * Math.sin(heading) + strafe * Math.cos(heading);
        double adjustedStrafe = vertical * Math.cos(heading) - strafe * Math.sin(heading);

//        double adjustedVertical = vertical * Math.cos(heading) - strafe * Math.sin(heading);
//        double adjustedStrafe = vertical * Math.sin(heading) + strafe * Math.cos(heading);
        Vector2d positionInput = new Vector2d(adjustedStrafe, adjustedVertical);

        drive.setDrivePowers(new PoseVelocity2d(
                positionInput,
                //turn
                -gamepad2.right_stick_x
        ));
        drive.updatePoseEstimate();


        if (gamepad2.b) {
            Actions.runBlocking(
                    drive.actionBuilder(pose)
                            .strafeToLinearHeading(new Vector2d(40,-30),Math.toRadians(360))
                            .build());
        } else if (gamepad2.x) {
            Actions.runBlocking(
                    drive.actionBuilder(pose)
                            .endTrajectory()
                            .build()
            );
        }

        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(heading));
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }
}