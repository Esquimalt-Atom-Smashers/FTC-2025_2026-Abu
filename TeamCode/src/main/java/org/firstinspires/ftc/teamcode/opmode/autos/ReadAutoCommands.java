package org.firstinspires.ftc.teamcode.opmode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.lang.reflect.Field;
import java.util.ArrayList;

@Autonomous(name = "Experimental: ReadAutoCommands")
public class ReadAutoCommands extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            ArrayList<String> actionLines = RobotPropertyParser.loadAuto();
            Field[] fields = Property.class.getDeclaredFields();

            for (Field field : fields) {
                try {
                    String key = field.getName();
                    Object value = field.get(null); // null because fields are static
                    telemetry.addLine(key + " = " + value);
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }

            telemetry.addLine("===================");

            for (String line : actionLines) {
                telemetry.addLine(line);
            }
            telemetry.update();
        }
    }
}

