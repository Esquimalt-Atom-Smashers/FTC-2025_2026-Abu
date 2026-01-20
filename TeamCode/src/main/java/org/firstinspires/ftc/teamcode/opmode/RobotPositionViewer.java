package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.RobotPositionHolder;
@TeleOp
public class RobotPositionViewer extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("press a to reset pos");
            telemetry.addData("x", RobotPositionHolder.getX());
            telemetry.addData("y", RobotPositionHolder.getY());
            telemetry.addData("heading", RobotPositionHolder.getHeading());
            telemetry.update();
            if (gamepad1.a) {
                RobotPositionHolder.clear();
            }
        }
    }
}
