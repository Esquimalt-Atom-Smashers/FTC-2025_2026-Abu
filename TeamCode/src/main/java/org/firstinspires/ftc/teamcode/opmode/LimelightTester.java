package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
import org.firstinspires.ftc.teamcode.utilities.CommandManager;
@TeleOp
@Config
public class LimelightTester extends LinearOpMode {
    private LimelightSybsystem limelightSybsystem;
    public static class Params {
            public static boolean isRedAlliance = false;
    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        limelightSybsystem = new LimelightSybsystem(this, PARAMS.isRedAlliance);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            limelightSybsystem.periodic();
            telemetry.addData("distance from goal", limelightSybsystem.getDistanceFromGoal());
            telemetry.addData("ty", limelightSybsystem.getTy());
            telemetry.addData("oblisk tag id" ,limelightSybsystem.getTagId());

            Pose2d currentPos = limelightSybsystem.getBotPose2D(new Pose2d(0,0,0), true);
            telemetry.addData("currentPosX", currentPos.position.x);
            telemetry.addData("currentPosY", currentPos.position.y);
            telemetry.addData("currentPosHeading", Math.toDegrees(currentPos.heading.toDouble()));

            telemetry.update();
        }
    }
}
