package org.firstinspires.ftc.teamcode.opmode.autos;

import static org.firstinspires.ftc.teamcode.utilities.RobotContainer.drivebase;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer.*;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.util.ArrayList;

@Autonomous(name = "Experimental: AutoFromTextFile")
public class AutoFromTextFile extends LinearOpMode {
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
        Alliance alliance;
        switch (startingPosTxt) {
            case "RED.FAR":
                startingPos = new Pose2d(65,16, Math.toRadians(180));
                alliance = Alliance.RED;
                telemetry.addLine("starting at RED.FAR");
                break;
            case "RED.CLOSE":
                startingPos = new Pose2d(-72 + 19.5, 72 - 18, Math.toRadians(53.5));
                alliance = Alliance.RED;
                telemetry.addLine("starting at RED.CLOSE");
            case "BLUE.FAR":
                startingPos = new Pose2d(72 - 7, -16, Math.toRadians(180));
                alliance = Alliance.BLUE;
                telemetry.addLine("starting at BLUE.FAR");
                break;
            case "BLUE.CLOSE":
                startingPos = new Pose2d(-72 + 21.5, -72 + 17.5, Math.toRadians(143.65));
                alliance = Alliance.BLUE;;
                telemetry.addLine("starting at BLUE.CLOSE");
                break;
            case "TESTING.MODE":
                startingPos = new Pose2d(0, 0, 0);
                alliance = Alliance.RED;
                telemetry.addLine("starting at MIDDLE.OF.FIELD(testing mode)");
                break;
            default:
                try {
                    throw new Exception("Invalid starting position");
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }}
        actions.remove(0);
        RobotContainer robotContainer = new RobotContainer(this,
                startingPos, alliance,
                DriveSubsystem.DriveSubsystemState.AUTO,
                ShooterSubsystem.ShooterState.DISABLED,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.DISABLED);
        //cycle through the actions to add them to the roadrunner action array
        for (String action: actions){
            switch (action) {
                case "RED.FAR.SHOOT":
                    actionList.add(redFarShootAction());
                    telemetry.addLine("RED.FAR.SHOOT");
                    break;
                case "RED.THIRD.INTAKE":
                    actionList.add(redThirdIntakeAction());
                    telemetry.addLine("RED.THIRD.INTAKE");
                    break;
                case "DELAY.HUNDRED.MS":
                    actionList.add(new SleepAction(0.1));
                    break;
                case "DELAY.ONE.S":
                    actionList.add(new SleepAction(1));
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
        Actions.runBlocking(
                new ParallelAction(actionSequence,
                        new InstantAction(robotContainer::runRobot))
        );
        robotContainer.shutDownRobot();
    }

    private Action redFarShootAction() {
        return drivebase.getMecanumDrive().actionBuilder(drivebase.getPose()).
                strafeToLinearHeading(new Vector2d(55, 18), Math.toRadians(72)).build();
    }

    private Action redThirdIntakeAction() {
        return drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                .strafeToLinearHeading(new Vector2d(40, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, 43), Math.toRadians(90))
                .build();
    }
    private Action printAction(String line) {
        return new InstantAction(() -> telemetry.addLine(line));

    }
}