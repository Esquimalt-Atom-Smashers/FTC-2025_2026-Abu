package org.firstinspires.ftc.teamcode.utilities;

import java.io.File;
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
import java.util.Scanner;

public class RobotPropertyParser {

    private static Properties robotProperties = new Properties();
    private static Properties autoSequence1Properties = new Properties();
    private static File autoSequenceTxt;
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
        autoSequenceTxt = new File(FILE_LOCATION + "/" + AUTO_SEQUENCE1_FILE_NAME);
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
    /**
     * Used to load values from the text file to the properties class
     */
    public static void populatePropertiesClass(){
        loadProperties();

        Field[] fields = Property.class.getDeclaredFields();
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

    public static ArrayList<String> arrayFromAutoSequenceFile(){
        loadAutoSequence1();
        ArrayList<String> actionList = new ArrayList<>();
        try(Scanner reader = new Scanner(autoSequenceTxt)) {
            while (reader.hasNextLine()) {
                String actionTxt = reader.nextLine();
                actionList.add(actionTxt);
            }
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        return actionList;
    }
}