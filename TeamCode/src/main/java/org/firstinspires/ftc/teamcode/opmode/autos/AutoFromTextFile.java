package org.firstinspires.ftc.teamcode.opmode.autos;

import static org.firstinspires.ftc.teamcode.utilities.Property.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.AutoActions;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer.*;
import org.firstinspires.ftc.teamcode.utilities.RobotPositionHolder;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.util.ArrayList;
@Disabled
@Autonomous(name = "Experimental: AutoFromTextFile")
public class AutoFromTextFile extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPos;

        //array list for roadrunner to control actions in sequence
        ArrayList<Action> actionList = new ArrayList<>();
        SequentialAction actionSequence;

        //get the actions from the ActiveAutoSequence & property form Txt file
        ArrayList<String> actions = RobotPropertyParser.arrayFromAutoSequenceFile();
        RobotPropertyParser.populatePropertiesClass();

        //Print auto title
        telemetry.addLine("Title: " + actions.get(0));
        actions.remove(0);

        //Get starting Position
        String startingPosTxt = actions.get(0);
        Alliance alliance;
        switch (startingPosTxt) {
            case "RED.FAR":
                startingPos = new Pose2d(RED_FAR_X, RED_FAR_Y, Math.toRadians(RED_FAR_HEADING));
                alliance = Alliance.RED;
                telemetry.addLine("starting at RED.FAR");
                break;
            case "RED.CLOSE":
                startingPos = new Pose2d(RED_CLOSE_X, RED_CLOSE_Y, Math.toRadians(RED_CLOSE_HEADING));
                alliance = Alliance.RED;
                telemetry.addLine("starting at RED.CLOSE");
            case "BLUE.FAR":
                startingPos = new Pose2d(BLUE_FAR_X, BLUE_FAR_Y, Math.toRadians(BLUE_FAR_HEADING));
                alliance = Alliance.BLUE;
                telemetry.addLine("starting at BLUE.FAR");
                break;
            case "BLUE.CLOSE":
                startingPos = new Pose2d(BLUE_CLOSE_X, BLUE_CLOSE_Y, Math.toRadians(BLUE_CLOSE_HEADING));
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
                ShooterSubsystem.ShooterState.MANUAL,
                IntakeTransferSubsystem.IntakeTransferState.INTAKING,
                VisionSubsystem.VisionState.DISABLED);
        AutoActions autoActions = new AutoActions(robotContainer);

        //cycle through the actions to add them to the roadrunner action array
        for (String action: actions){
            switch (action) {
                case "RED.FAR.SHOOT":
                    actionList.add(autoActions.redFarShootAction());
                    actionList.add(autoActions.shootArtifactAction(FAR_SHOOT_RPM, SHOOTING_SECONDS));
                    telemetry.addLine("RED.FAR.SHOOT");
                    break;
                case "RED.THIRD.INTAKE":
                    actionList.add(autoActions.redThirdIntakeAction());
                    telemetry.addLine("RED.THIRD.INTAKE");
                    break;
                case "RED.LOAD.INTAKE":
                    actionList.add(autoActions.redLoadingZoneIntakeAction());
                    telemetry.addLine("RED.LOAD.INTAKE");
                    break;
                case "UPDATE.POSE":
                    actionList.add(autoActions.updatePoseFromVisionAction(false));
                    telemetry.addLine("UPDATE.POSE");
                case "FORCED.UPDATE.POSE":
                    actionList.add(autoActions.updatePoseFromVisionAction(true));
                    telemetry.addLine("UPDATE.POSE.FORCED");
                case "DELAY.HUNDRED.MS":
                    actionList.add(new SleepAction(0.1));
                    break;
                case "DELAY.ONE.S":
                    actionList.add(new SleepAction(1));
                    break;
                default:
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
        Pose2d lastPose = robotContainer.getPose();
        RobotPositionHolder.storePos(lastPose.position.x, lastPose.position.y, lastPose.heading.toDouble());
        robotContainer.shutDownRobot();
    }


}