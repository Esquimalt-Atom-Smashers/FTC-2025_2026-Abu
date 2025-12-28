package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Properties;
import org.firstinspires.ftc.teamcode.RobotPropertyParser;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.Scanner;

@TeleOp
public class ReadText extends LinearOpMode {
    public void runOpMode() {
        Exception e1 = new Exception("none");
        java.util.Properties props = new java.util.Properties();

        waitForStart();
        while (opModeIsActive()) {
//            try {
//                props.load(new FileInputStream(RobotPropertyParser.FILE_LOCATION + "/"+ RobotPropertyParser.FILE_NAME));
//            } catch (Exception e) {
//                e1 = e;
//                telemetry.addData("bad exception",e);
//            }
            RobotPropertyParser.populateConstantsClass(this.telemetry);
//            String appVersion = props.getProperty("STRAFE_OFFSET");
//            if (appVersion != null) {
//                telemetry.addData("strafing offset", appVersion);
//            } else {
//                telemetry.addData("exception", e1);
//            }
            telemetry.addData("strafe offset", Properties.STRAFE_OFFSET);
            telemetry.update();
        }
        RobotPropertyParser.populatePropertiesFile();
    }
}
