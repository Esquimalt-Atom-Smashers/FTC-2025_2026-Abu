package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(0));
        driveSubsystem = new DriveSubsystem(this, startingPose);
        RobotPropertyParser.populateAutoSequenceClass();


        //array list for roadrunner to control actions in sequence
        ArrayList<Action> actionList = new ArrayList<>();
        SequentialAction actionSequence;

        //get the actions from the ActiveAutoSequence
        ArrayList<String> actions = RobotPropertyParser.arrayFromAutoSequenceFile();
        //cycle through the actions to add them to the roadrunner action array
        for (String action: actions){
            switch (action) {
                case "TO_POS1":
                    //actionList.add(goToPos1());
                    telemetry.addLine("action to pos 1");
                    break;
                case "TO_POS2":
                    //actionList.add(goToPos2());
                    telemetry.addLine("action to pos 2");
                    break;
                default:
                    //printAction("default liner");
                    telemetry.addLine("default");
                    break;
            }
        }

        telemetry.update();

        waitForStart();
        //once robot starts do the prescribed actions from the action list.
        actionSequence = new SequentialAction(actionList);
        Actions.runBlocking(actionSequence);
        //RobotPropertyParser.populateAutoSequenceFile();
    }

    private Action goToPos1Action() {
//        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
//                strafeToLinearHeading(new Vector2d(10, 0), Math.toRadians(90)).build();
        return new InstantAction(() -> telemetry.addLine("action1 go to pos 1"));
    }

    private Action goToPos2Action() {
//        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
//                strafeToLinearHeading(new Vector2d(0, 10), Math.toRadians(0)).build();
        return new InstantAction(() -> telemetry.addLine("action2 go to pos 2"));

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
