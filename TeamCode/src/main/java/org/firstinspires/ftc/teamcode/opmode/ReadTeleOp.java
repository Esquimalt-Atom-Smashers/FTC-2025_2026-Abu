package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "ReadTeleOp (Blocks to Java)")
public class ReadTeleOp extends LinearOpMode {

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (RobotContainer.hasData()) {
                    telemetry.addData("has data", 123);
                } else {
                    telemetry.addData("no data", 123);
                }
                if (gamepad1.a) {
                    RobotContainer.markReceived();
                }
                telemetry.addData("x", RobotContainer.getX());
                telemetry.addData("y", RobotContainer.getY());
                telemetry.addData("a", RobotContainer.getHeading());
                telemetry.update();
            }
            RobotContainer.clear();
        }
    }
}