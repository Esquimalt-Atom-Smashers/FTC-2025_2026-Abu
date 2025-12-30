package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ActiveAutoSequence;
import org.firstinspires.ftc.teamcode.Properties;
import org.firstinspires.ftc.teamcode.RobotPropertyParser;

@TeleOp
public class ReadText extends LinearOpMode {
    public void runOpMode() {
        Exception e1 = new Exception("none");
        java.util.Properties props = new java.util.Properties();
        RobotPropertyParser.populatePropertiesClass();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("strafe offset", Properties.STRAFE_OFFSET);
            telemetry.addData("action1", ActiveAutoSequence.Action1);
            telemetry.addData("action2", ActiveAutoSequence.Action2);
            telemetry.update();
        }
        RobotPropertyParser.populatePropertiesFile();
    }
}
