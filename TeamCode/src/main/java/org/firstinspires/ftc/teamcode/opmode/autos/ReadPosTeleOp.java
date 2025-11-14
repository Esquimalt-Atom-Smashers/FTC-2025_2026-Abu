package org.firstinspires.ftc.teamcode.opmode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class ReadPosTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            RobotContainer.markReceived();

            telemetry.addData("data is valid", RobotContainer.hasData());
            telemetry.addData("x", RobotContainer.getX());
            telemetry.addData("y", RobotContainer.getY());
            telemetry.addData("heading", RobotContainer.getHeading());
        }

        RobotContainer.clear();
    }
}
