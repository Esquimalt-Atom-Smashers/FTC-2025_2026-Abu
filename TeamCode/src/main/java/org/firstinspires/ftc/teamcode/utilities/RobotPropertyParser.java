package org.firstinspires.ftc.teamcode.utilities;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Properties;

public class RobotPropertyParser {

    private static Properties robotProperties = new Properties();
    private static Properties autoSequence1Properties = new Properties();
    private static File autoSequenceTxt;
    public static final String FILE_LOCATION = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode";
    public static final String PROPERTIES_FILE_NAME = "robot_properties.txt";
    public static final String CUSTOMIZABLE_AUTO_FILE_NAME = "ActiveAuto.txt";
    public static final String RED_FAR_AUTO_FILE_NAME = "RedFarAuto.++txt";
    public static final String RED_FAR_THIRD_AUTO_FILE_NAME = "RedFarThirdAuto.txt";
    public static final String RED_CLOSE_AUTO_FILE_NAME = "RedCloseAuto.txt";
    public static final String BLUE_FAR_AUTO_FILE_NAME = "BlueFarAuto.txt";
    public static final String BLUE_FAR_THIRD_AUTO_FILE_NAME = "BlueFarThirdAuto.txt";
    public static final String BLUE_CLOSE_AUTO_FILE_NAME = "BlueCloseAuto.txt";
//skibidi
    public enum AUTO {
        CUSTOMIZABLE_AUTO(CUSTOMIZABLE_AUTO_FILE_NAME),
        RED_FAR_AUTO(RED_FAR_AUTO_FILE_NAME),

        RED_FAR_THIRD_AUTO(RED_FAR_THIRD_AUTO_FILE_NAME),
        RED_CLOSE_AUTO(RED_CLOSE_AUTO_FILE_NAME),
        BLUE_FAR_AUTO(BLUE_FAR_AUTO_FILE_NAME),
        BLUE_FAR_THIRD_AUTO(BLUE_FAR_THIRD_AUTO_FILE_NAME),
        BLUE_CLOSE_AUTO(BLUE_CLOSE_AUTO_FILE_NAME);

        private final String fileName;

        AUTO(String fileName) {
            this.fileName = fileName;
        }

        public String getFileName() {
            return fileName;
        }
    }


    public static void loadProperties() {
        try {
            robotProperties.clear();
            robotProperties.load(new FileInputStream(FILE_LOCATION + "/"+ PROPERTIES_FILE_NAME));
        } catch (Exception e) {
            robotProperties = new Properties();
        }
    }

    public static void loadAutoSequence1() {
        autoSequenceTxt = new File(FILE_LOCATION + "/" + CUSTOMIZABLE_AUTO_FILE_NAME);
    }

    /**
     * Gets a double from the robot_properties.txt file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static double getDouble(String key, Properties propertiesFile) { return Double.parseDouble(propertiesFile.getProperty(key)); }

    /**
     * Gets an integer for the robot_properties.txt file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static int getInt(String key, Properties propertiesFile) { return Integer.parseInt(propertiesFile.getProperty(key)); }

    public static String getString(String key, Properties propertiesFile) { return propertiesFile.getProperty(key);}

    /**
     * Used to make EXTRA separated back-up files in OnBot
     */
    public static void saveProperties(){
        try {
            // save the current properties to a back up file
            Date d = new Date();
            SimpleDateFormat format = new SimpleDateFormat("yyyy-M-dd-hh-mm");
            OutputStream backupOutputStream = new FileOutputStream(FILE_LOCATION +"/backup-"+format.format(d)+".properties");

            robotProperties.store(backupOutputStream,null);
            backupOutputStream.flush();
            backupOutputStream.close();

            populatePropertiesFile();
            OutputStream newOutputStream = new FileOutputStream(FILE_LOCATION + "/"+ PROPERTIES_FILE_NAME);

            robotProperties.store(newOutputStream,null);
            newOutputStream.flush();
            newOutputStream.close();

        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Used to save current changes into the text files
     */
    public static void populatePropertiesFile(){

        Field[] fields = Property.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                try {
                    if(clazz.isAssignableFrom(double.class)) {
                        double value = field1.getDouble(null);
                        robotProperties.setProperty(fieldName,Double.toString(value));
                    }
                    if(clazz.isAssignableFrom(int.class)){
                        int value = field1.getInt(null);
                        robotProperties.setProperty(fieldName,Integer.toString(value));
                    }
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public static void loadTeleOp() {
        loadFile(FILE_LOCATION + "/" + PROPERTIES_FILE_NAME, false);
    }

    // =========================
    // AUTO LOADING
    // =========================
    public static ArrayList<String> loadAuto(AUTO auto) {
        // Load base config first
        loadTeleOp();
        // Then override with auto config
        return loadFile(FILE_LOCATION + "/" + auto.fileName, true);

    }

    // =========================
    // SHARED FILE LOADER
    // =========================
    private static ArrayList<String> loadFile(String filePath, boolean allowCommands) {
        ArrayList<String> actionList = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();

                if (line.isEmpty()) {
                    continue;
                }

                // =========================
                // PROPERTY LINE
                // =========================
                if (line.contains("=")) {
                    String[] parts = line.split("=");
                    String key = parts[0].trim();
                    String value = parts[1].trim();

                    // pose2d override
                    if (value.startsWith("pose2d")) {
                        String inside = value.substring(
                                value.indexOf("(") + 1,
                                value.indexOf(")")
                        );

                        String[] nums = inside.split(",");
                        double x = Double.parseDouble(nums[0].trim());
                        double y = Double.parseDouble(nums[1].trim());
                        double heading = Double.parseDouble(nums[2].trim());

                        String fieldName = key.replace(".", "_");

                        setField(fieldName + "_X", x);
                        setField(fieldName + "_Y", y);
                        setField(fieldName + "_HEADING", heading);

//                        if (allowCommands) {
//                            System.out.println("Override pose2d -> x:" + x +
//                                    " y:" + y + " heading:" + heading);
//                        }
                    }
                    // numeric property
                    else {
                        double number = Double.parseDouble(value);
                        String fieldName = key.replace(".", "_");

                        setField(fieldName, number);

                        if (allowCommands) {
                            System.out.println("Override " + fieldName + " = " + number);
                        }
                    }
                } else if (allowCommands) {
                    actionList.add(line);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return actionList;
    }

    // =========================
    // REFLECTION SETTER
    // =========================
    private static void setField(String fieldName, double value) {
        try {
            Field field = Property.class.getDeclaredField(fieldName);
            field.setAccessible(true);
            field.setDouble(null, value);
        } catch (NoSuchFieldException e) {
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }
}