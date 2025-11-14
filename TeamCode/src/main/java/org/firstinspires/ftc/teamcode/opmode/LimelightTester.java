package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
@TeleOp
public class LimelightTester extends LinearOpMode {
    private LimelightSybsystem limelightSybsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        limelightSybsystem = new LimelightSybsystem(this, true);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            limelightSybsystem.periodic();
            telemetry.addData("distance from goal", limelightSybsystem.getDistanceFromGoal());
            telemetry.addData("ty", limelightSybsystem.getTy());
            telemetry.addData("oblisk tag id" ,limelightSybsystem.getTagId());

            Pose2d currentPos = limelightSybsystem.getBotPose2D(new Pose2d(0,0,0));
            telemetry.addData("currentPos", currentPos.position.x);
            telemetry.addData("currentPos", currentPos.position.y);
            telemetry.addData("currentPos", Math.toDegrees(currentPos.heading.toDouble()));

            telemetry.update();
        }
    }
}
