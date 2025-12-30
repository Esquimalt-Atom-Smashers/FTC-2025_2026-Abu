package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActiveAutoSequence;
import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

@Autonomous(name = "Experimental: AutoFromTextFile")
public class AutoFromTextFile extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPos;

        //array list for roadrunner to control actions in sequence
        ArrayList<Action> actionList = new ArrayList<>();
        SequentialAction actionSequence;

        //get the actions from the ActiveAutoSequence
        ArrayList<String> actions = RobotPropertyParser.arrayFromAutoSequenceFile();

        //Print auto title
        telemetry.addLine("Title: " + actions.get(0));
        actions.remove(0);

        //Get starting Position
        String startingPosTxt = actions.get(0);
        switch (startingPosTxt) {
            case "BLUE_FAR":
                startingPos = new Pose2d(72 - 7, -16, Math.toRadians(180));
                telemetry.addLine("starting at BLUE_FAR");
                break;
            case "BLUE_CLOSE":
                startingPos = new Pose2d(-72 + 21.5, -72 + 17.5, Math.toRadians(143.65));
                telemetry.addLine("starting at BLUE_CLOSE");
                break;
            case "RED_FAR":
                startingPos = new Pose2d(65,16, Math.toRadians(180));
                telemetry.addLine("starting at RED_FAR");
                break;
            case "RED_CLOSE":
                startingPos = new Pose2d(-72 + 19.5, 72 - 18, Math.toRadians(53.5));
                telemetry.addLine("starting at RED_CLOSE");
                break;
            case "TESTING_MODE":
                startingPos = new Pose2d(0, 0, 0);
                telemetry.addLine("starting at MIDDLE_OF_FIELD(testing mode)");
                break;
            default:
                try {
                    throw new Exception("Invalid starting position");
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }}
        actions.remove(0);
        driveSubsystem = new DriveSubsystem(this, startingPos);

        //cycle through the actions to add them to the roadrunner action array
        for (String action: actions){
            switch (action) {
                case "TO_POS1":
                    actionList.add(goToPos1Action());
                    telemetry.addLine("action to pos 1");
                    break;
                case "TO_POS2":
                    actionList.add(goToPos2Action());
                    telemetry.addLine("action to pos 2");
                    break;
                default:
                    printAction("default liner");
                    telemetry.addLine("I read:" + action);
                    break;
            }
        }

        telemetry.update();
        waitForStart();
        //once robot starts do the prescribed actions from the action list.
        actionSequence = new SequentialAction(actionList);
        Actions.runBlocking(actionSequence);
    }

    private Action goToPos1Action() {
        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
                strafeToLinearHeading(new Vector2d(10, 0), Math.toRadians(90)).build();
//        return new InstantAction(() -> telemetry.addLine("action1 go to pos 1"));
    }

    private Action goToPos2Action() {
        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
                strafeToLinearHeading(new Vector2d(0, 10), Math.toRadians(0)).build();
//        return new InstantAction(() -> telemetry.addLine("action2 go to pos 2"));

    }
    private Action printAction(String line) {
//        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
//                strafeToLinearHeading(new Vector2d(0, 10), Math.toRadians(0)).build();
        return new InstantAction(() -> telemetry.addLine(line));

    }

    private Action printNum(int i) {
        return new InstantAction(() -> telemetry.addData("i=",i));
    }
}
