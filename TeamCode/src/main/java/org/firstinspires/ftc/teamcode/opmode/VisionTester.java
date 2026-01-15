package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
@TeleOp
@Config
public class VisionTester extends LinearOpMode {
    public static class Params {
        public double currentHeading = 0.0;
    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystem visionSubsystem = new VisionSubsystem(this, RobotContainer.Alliance.BLUE, VisionSubsystem.VisionState.TRACKING_GOAL, new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(telemetry);
        waitForStart();
        while (opModeIsActive()) {
            visionSubsystem.updateCurrentPose(new Pose2d(0,0, Math.toRadians(PARAMS.currentHeading)));
            visionSubsystem.periodic();
            visionSubsystem.enableSubsystemTelemetry(false);
            Pose2d visionPos = visionSubsystem.getLimelightPos();
            if (visionPos != null) {
                telemetry.addData("x", visionPos.position.x);
                telemetry.addData("y", visionPos.position.y);
                telemetry.addData("heading", Math.toDegrees(visionPos.heading.toDouble()));
            } else {
                telemetry.addLine("no LL data");
            }
            telemetry.update();
        }
    }
}
