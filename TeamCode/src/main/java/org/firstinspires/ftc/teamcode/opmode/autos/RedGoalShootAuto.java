package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoSequence1;
import org.firstinspires.ftc.teamcode.Properties;
import org.firstinspires.ftc.teamcode.RobotPropertyParser;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Objects;

@Autonomous(name = "Experimental: RedGoalShootAuto")
public class RedGoalShootAuto extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(0));
        driveSubsystem = new DriveSubsystem(this, startingPose);
        RobotPropertyParser.populateAutoSequenceClass(telemetry);

        ArrayList<Action> actionList = new ArrayList<>();
        SequentialAction actionSequence;
        for (int i=1; i<=10; i++) {
            String fieldName = "Action" + i;
            try {
                Field field = AutoSequence1.class.getDeclaredField(fieldName);
                field.setAccessible(true);

                String value = (String)field.get(null);
                if (value == null) break;

                switch (value) {
                    case "TO_POS1":
                        actionList.add(goToPos1());
                        break;
                    case "TO_POS2":
                        actionList.add(goToPos2());
                        break;
                    default:
                        break;
                }

            } catch (NoSuchFieldException e) {
                break;
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        waitForStart();
        actionSequence = new SequentialAction(actionList);
        Actions.runBlocking(actionSequence);
        RobotPropertyParser.populateAutoSequenceFile();
    }

    private Action goToPos1() {
        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
                strafeToLinearHeading(new Vector2d(10, 0), Math.toRadians(90)).build();
    }

    private Action goToPos2() {
        return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos()).
                strafeToLinearHeading(new Vector2d(0, 10), Math.toRadians(0)).build();
    }
}
