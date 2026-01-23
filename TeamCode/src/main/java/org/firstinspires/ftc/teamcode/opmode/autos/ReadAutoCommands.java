package org.firstinspires.ftc.teamcode.opmode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.lang.reflect.Field;
import java.util.ArrayList;

@Autonomous(name = "ReadAutoCommands")
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
            ArrayList<String> actionLines = RobotPropertyParser.loadAuto(RobotPropertyParser.AUTO.CUSTOMIZABLE_AUTO);
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

//    public void loadTeleOp() {
//        loadFile(FILE_LOCATION + "/" + PROPERTIES_FILE_NAME, false);
//    }
//    public ArrayList<String> loadAuto() {
//        // Load base config first
//        loadTeleOp();
//        // Then override with auto config
//        return loadFile(FILE_LOCATION + "/" + AUTO_SEQUENCE1_FILE_NAME, true);
//
//    }
//
//    // =========================
//    // SHARED FILE LOADER
//    // =========================
//    private ArrayList<String> loadFile(String filePath, boolean allowCommands) {
//        ArrayList<String> actionList = new ArrayList<>();
//
//        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
//            String line;
//            while ((line = br.readLine()) != null) {
//                line = line.trim();
//
//                if (line.isEmpty()) {
//                    continue;
//                }
//
//                // =========================
//                // PROPERTY LINE
//                // =========================
//                if (line.contains("=")) {
//                    String[] parts = line.split("=");
//                    String key = parts[0].trim();
//                    String value = parts[1].trim();
//
//                    // pose2d override
//                    if (value.startsWith("pose2d")) {
//                        telemetry.addLine("detect a pose2d");
//                        String inside = value.substring(
//                                value.indexOf("(") + 1,
//                                value.indexOf(")")
//                        );
//
//                        String[] nums = inside.split(",");
//                        double x = Double.parseDouble(nums[0].trim());
//                        double y = Double.parseDouble(nums[1].trim());
//                        double heading = Double.parseDouble(nums[2].trim());
//
//                        setField("RED_GOAL_X", x);
//                        setField("RED_GOAL_Y", y);
//                        setField("RED_GOAL_HEADING", heading);
//
////                        if (allowCommands) {
////                            System.out.println("Override pose2d -> x:" + x +
////                                    " y:" + y + " heading:" + heading);
////                        }
//                    }
//                    // numeric property
//                    else {
//                        telemetry.addLine("detect double");
//                        double number = Double.parseDouble(value);
//                        String fieldName = key.replace(".", "_");
//
//                        setField(fieldName, number);
//
//                        if (allowCommands) {
//                            telemetry.addLine("Override " + fieldName + " = " + number);
//                        }
//                    }
//                } else if (allowCommands) {
//                    actionList.add(line);
//                }
//
//                // =========================
//                // AUTO COMMANDS
//                // =========================
////                else if (allowCommands && line.equals("FAR.SHOOT.POS")) {
////                    System.out.println("I am doing FAR.SHOOT.POS command");
////                }
////
////                else if (allowCommands && line.startsWith("GO.TO.POSE2D")) {
////                    String inside = line.substring(
////                            line.indexOf("(") + 1,
////                            line.indexOf(")")
////                    );
////
////                    String[] nums = inside.split(",");
////                    double x = Double.parseDouble(nums[0].trim());
////                    double y = Double.parseDouble(nums[1].trim());
////                    double heading = Double.parseDouble(nums[2].trim());
////
////                    System.out.println(
////                            "I am going to " + x + "," + y + "," + heading
////                    );
////                }
//
//                // =========================
//                // UNKNOWN / TYPO
//                // =========================
//            }
//
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//
//        return actionList;
//    }
//
//    // =========================
//    // REFLECTION SETTER
//    // =========================
//    private void setField(String fieldName, double value) {
//        try {
//            Field field = Property.class.getDeclaredField(fieldName);
//            telemetry.addLine("found field" + fieldName);
//            field.setAccessible(true);
//            field.setDouble(null, value);
//        } catch (NoSuchFieldException e) {
//            telemetry.addLine("cannot change double " + fieldName);
//        } catch (IllegalAccessException e) {
//            e.printStackTrace();
//        }
//    }


}

