package org.firstinspires.ftc.teamcode;


import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
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
    private static final String FILE_LOCATION = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode";
    private static final String PROPERTIES_FILE_NAME = "robot_properties.txt";
    private static final String AUTO_SEQUENCE1_FILE_NAME = "ActiveAuto.txt";

    public static void loadProperties() {
        try {
            robotProperties.clear();
            robotProperties.load(new FileInputStream(FILE_LOCATION + "/"+ PROPERTIES_FILE_NAME));
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
    /**
     * Used to load values from the text file to the properties class
     */
    public static void populatePropertiesClass(){
        loadProperties();

        Field[] fields = org.firstinspires.ftc.teamcode.Properties.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                if(robotProperties.containsKey(fieldName)){
                    try {
                        if(clazz.isAssignableFrom(double.class)) {
                            field1.setDouble(fieldName,getDouble(fieldName,robotProperties));
                        }
                        if(clazz.isAssignableFrom(int.class)){
                            field1.setInt(fieldName,getInt(fieldName,robotProperties));
                        }
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }

    public static void populateAutoSequenceFile(){

        Field[] fields = ActiveAutoSequence.class.getDeclaredFields();
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
                        String value = field1.get(null).toString();
                        autoSequence1Properties.setProperty(fieldName, value);
                    }
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
    public static ArrayList<String> arrayFromAutoSequenceFile(){

        Field[] fields = ActiveAutoSequence.class.getDeclaredFields();

        ArrayList<String> actionList = new ArrayList<>();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                Class clazz = field1.getType();
                try {
                    if(clazz.isAssignableFrom(String.class)) {
                        String value = field1.get(null).toString();
                        if (!value.equals("default")){
                            actionList.add(value);
                        }

                    }
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return actionList;
    }
    public static void populateAutoSequenceClass(){
        loadAutoSequence1();

        Field[] fields = ActiveAutoSequence.class.getDeclaredFields();
        for (Field field1: fields){
            if(Modifier.isStatic(field1.getModifiers())){
                String fieldName = field1.getName();
                Class clazz = field1.getType();
                if(autoSequence1Properties.containsKey(fieldName)){
                    try {
                        if(clazz.isAssignableFrom(double.class)) {
                            field1.setDouble(fieldName,getDouble(fieldName,autoSequence1Properties));
                        }
                        if(clazz.isAssignableFrom(int.class)){
                            field1.setInt(fieldName,getInt(fieldName,autoSequence1Properties));
                        }
                        if(clazz.isAssignableFrom(String.class)) {
                            field1.set(fieldName, autoSequence1Properties.getProperty(fieldName));
                        }
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }
}
