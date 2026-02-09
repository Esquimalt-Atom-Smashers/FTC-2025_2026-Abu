package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoMotorShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.AutoActions;
import org.firstinspires.ftc.teamcode.utilities.AutoCommandInterpreter;
import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.util.ArrayList;

@Autonomous(name = "Blue Far Auto")
public class BlueFarAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //array list for roadrunner to control actions in sequence
        ArrayList<Action> actionList;
        SequentialAction actionSequence;

        //get the actions from the ActiveAutoSequence & property form Txt file
        AutoCommandInterpreter interpreter = new AutoCommandInterpreter(RobotPropertyParser.loadAuto(RobotPropertyParser.AUTO.BLUE_FAR_AUTO));
        //Print auto title
        telemetry.addLine("Title: " + interpreter.readAndRemoveTitle());

        //Get starting Position
        interpreter.readAndRemoveStartingPos();
        telemetry.addLine(interpreter.getStartingPositionTelemetry());
        RobotContainer robotContainer = new RobotContainer(this,
                interpreter.getStartingPos(), interpreter.getAlliance(),
                DriveSubsystem.DriveSubsystemState.AUTO,
                TwoMotorShooterSubsystem.ShooterState.DISABLED,
                TurretSubsystem.TurretState.GOAL_LOCK,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.TRACKING_GOAL);
        AutoActions autoActions = new AutoActions(robotContainer);
        interpreter.readAutoCommends(autoActions);

        //cycle through the actions to add them to the roadrunner action array
        actionList = interpreter.getActionList();
        ArrayList<String> autoCommentTelemetry = interpreter.getAutoCommendTelemetry();
        for (String telemetryLine: autoCommentTelemetry) {
            telemetry.addLine(telemetryLine);
        }
        telemetry.update();
        waitForStart();
        robotContainer.shoot();
        //once robot starts do the prescribed actions from the action list.
        actionSequence = new SequentialAction(actionList);
        try {
            Actions.runBlocking(
                    new ParallelAction(actionSequence,
                            new InstantAction(robotContainer::runRobot),
                            new InstantAction(robotContainer::updatePositionHolderAuto)
                    )
            );
        } finally {
            robotContainer.shutDownRobot();
            robotContainer.updatePositionHolderAuto();
            Property.reset();
        }
    }
}