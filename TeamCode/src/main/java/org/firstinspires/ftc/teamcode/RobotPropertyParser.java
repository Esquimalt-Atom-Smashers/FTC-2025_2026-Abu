package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Properties;

public class RobotPropertyParser {

    private static Properties robotProperties = new Properties();
    private static Properties autoSequence1Properties = new Properties();
    private static final String FILE_LOCATION = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode";
    private static final String FILE_NAME = "robot_properties.txt";
    private static final String AUTO_SEQUENCE1_FILE_NAME = "auto_sequence1.txt";

    public static void loadProperties() {
        try {
            robotProperties.clear();
            robotProperties.load(new FileInputStream(FILE_LOCATION + "/"+ FILE_NAME));
        } catch (Exception e) {
            robotProperties = new Properties();
        }
    }

    public static void loadAutoSequence1() {
        try {
            autoSequence1Properties.clear();
            autoSequence1Properties.load(new FileInputStream(FILE_LOCATION + "/"+ AUTO_SEQUENCE1_FILE_NAME));
        } catch (Exception e) {
            autoSequence1Properties = new Properties();
        }
    }

    /**
     * Gets a double from the robot_properties.txt file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static double getDouble(String key) { return Double.parseDouble(robotProperties.getProperty(key)); }

    /**
     * Gets an integer for the robot_properties.txt file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static int getInt(String key) { return Integer.parseInt(robotProperties.getProperty(key)); }

    public static String getString(String key) { return autoSequence1Properties.getProperty(key);}

    /**
     * Used to make separated back-up files in OnBot
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
            OutputStream newOutputStream = new FileOutputStream(FILE_LOCATION + "/"+ FILE_NAME);

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
     * Used to save current changes into current files
     */
    public static void populatePropertiesFile(){

        Field[] fields = org.firstinspires.ftc.teamcode.Properties.class.getDeclaredFields();
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
    public static void populateConstantsClass(Telemetry telemetry){
        loadProperties();

        Field[] fields = org.firstinspires.ftc.teamcode.Properties.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                if(robotProperties.containsKey(fieldName)){
                    try {
                        if(clazz.isAssignableFrom(double.class)) {
                            field1.setDouble(fieldName,getDouble(fieldName));
                        }
                        if(clazz.isAssignableFrom(int.class)){
                            field1.setInt(fieldName,getInt(fieldName));
                        }
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }

    public static void populateAutoSequenceFile(){

        Field[] fields = AutoSequence1.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                try {
                    if(clazz.isAssignableFrom(double.class)) {
                        double value = field1.getDouble(null);
                        autoSequence1Properties.setProperty(fieldName,Double.toString(value));
                    }
                    if(clazz.isAssignableFrom(int.class)){
                        int value = field1.getInt(null);
                        autoSequence1Properties.setProperty(fieldName,Integer.toString(value));
                    }
                    if(clazz.isAssignableFrom(String.class)) {
                        String content = autoSequence1Properties.getProperty(fieldName);
                        field1.set(fieldName, content);
                    }
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
    public static void populateAutoSequenceClass(Telemetry telemetry){
        loadProperties();

        Field[] fields = AutoSequence1.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                if(autoSequence1Properties.containsKey(fieldName)){
                    try {
                        if(clazz.isAssignableFrom(double.class)) {
                            field1.setDouble(fieldName,getDouble(fieldName));
                        }
                        if(clazz.isAssignableFrom(int.class)){
                            field1.setInt(fieldName,getInt(fieldName));
                        }
                        if(clazz.isAssignableFrom(String.class)) {
                            String content = autoSequence1Properties.getProperty(fieldName);
                            field1.set(fieldName, content);
                        }
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }
}
