package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;

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

            telemetry.update();
        }
    }
}
