package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_CLOSE_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_CLOSE_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_CLOSE_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_FAR_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_FAR_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_FAR_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_CLOSE_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_CLOSE_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_CLOSE_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_Y;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class AutoCommandInterpreter {
    private ArrayList<String> autoFileContent;
    private Pose2d startingPos;
    private RobotContainer.Alliance alliance;
    private String startingPositionTelemetry;
    private ArrayList<Action> actionList;
    private ArrayList<String> autoCommendTelemetry;
    public AutoCommandInterpreter(ArrayList<String> autoFileContent) {
        this.autoFileContent = autoFileContent;
    }
    public String readAndRemoveTitle() {
        String title = autoFileContent.get(0);
        autoFileContent.remove(0);
        return title;
    }
    public void readAndRemoveStartingPos() {
        String startingPosTxt = autoFileContent.get(0);
        switch (startingPosTxt) {
            case "RED.FAR":
                startingPos = new Pose2d(RED_FAR_X, RED_FAR_Y, Math.toRadians(RED_FAR_HEADING));
                alliance = RobotContainer.Alliance.RED;
                startingPositionTelemetry = "starting at RED.FAR";
                break;
            case "RED.CLOSE":
                startingPos = new Pose2d(RED_CLOSE_X, RED_CLOSE_Y, Math.toRadians(RED_CLOSE_HEADING));
                alliance = RobotContainer.Alliance.RED;
                startingPositionTelemetry = "starting at RED.CLOSE";
                break;
            case "BLUE.FAR":
                startingPos = new Pose2d(BLUE_FAR_X, BLUE_FAR_Y, Math.toRadians(BLUE_FAR_HEADING));
                alliance = RobotContainer.Alliance.BLUE;
                startingPositionTelemetry = "starting at BLUE.FAR";
                break;
            case "BLUE.CLOSE":
                startingPos = new Pose2d(BLUE_CLOSE_X, BLUE_CLOSE_Y, Math.toRadians(BLUE_CLOSE_HEADING));
                alliance = RobotContainer.Alliance.BLUE;;
                startingPositionTelemetry = "starting at BLUE.CLOSE";
                break;
            case "TESTING.MODE":
                startingPos = new Pose2d(0, 0, 0);
                alliance = RobotContainer.Alliance.RED;
                startingPositionTelemetry = "starting at MIDDLE.OF.FIELD(testing mode)";
                break;
            default:
                try {
                    throw new Exception("Invalid starting position");
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
        }
        autoFileContent.remove(0);
    }

    public void readAutoCommends(AutoActions autoActions) {
        for (String action: autoFileContent){
            switch (action) {
                case "RED.FAR.SHOOT":
                    actionList.add(autoActions.redFarShootAction());
                    autoCommendTelemetry.add("RED.FAR.SHOOT");
                    break;
                case "RED.CLOSE.SHOOT":
                    actionList.add(autoActions.redCloseShootAction());
                    autoCommendTelemetry.add("RED.CLOSE.SHOOT");
                    break;
                case "BLUE.FAR.SHOOT":
                    actionList.add(autoActions.blueFarShootAction());
                    autoCommendTelemetry.add("BLUE.FAR.SHOOT");
                    break;
                case "BLUE.CLOSE.SHOOT":
                    actionList.add(autoActions.blueCloseShootAction());
                    autoCommendTelemetry.add("BLUE.CLOSE.SHOOT");
                    break;
                case "RED.FIRST.INTAKE":
                    actionList.add(autoActions.redFirstIntakeAction());
                    autoCommendTelemetry.add("RED.FIRST.INTAKE");
                    break;
                case "RED.SECOND.INTAKE":
                    actionList.add(autoActions.redSecondIntakeAction());
                    autoCommendTelemetry.add("RED.SECOND.INTAKE");
                    break;
                case "RED.THIRD.INTAKE":
                    actionList.add(autoActions.redThirdIntakeAction());
                    autoCommendTelemetry.add("RED.THIRD.INTAKE");
                    break;
                case "RED.OPEN.GATE":
                    actionList.add(autoActions.redOpenGateAction());
                    autoCommendTelemetry.add("RED.OPEN.GATE");
                case "RED.GATE.INTAKE":
                    actionList.add(autoActions.redGateIntakeAction());
                    autoCommendTelemetry.add("RED.GATE.INTAKE");
                    break;
                case "RED.LOAD.INTAKE":
                    actionList.add(autoActions.redLoadingZoneIntakeAction());
                    autoCommendTelemetry.add("RED.LOAD.INTAKE");
                    break;
                case "BLUE.FIRST.INTAKE":
                    actionList.add(autoActions.blueFirstIntakeAction());
                    autoCommendTelemetry.add("BLUE.FIRST.INTAKE");
                    break;
                case "BLUE.SECOND.INTAKE":
                    actionList.add(autoActions.blueSecondIntakeAction());
                    autoCommendTelemetry.add("BLUE.SECOND.INTAKE");
                    break;
                case "BLUE.THIRD.INTAKE":
                    actionList.add(autoActions.blueThirdIntakeAction());
                    autoCommendTelemetry.add("BLUE.THIRD.INTAKE");
                    break;
                case "BLUE.OPEN.GATE":
                    actionList.add(autoActions.blueOpenGateAction());
                    autoCommendTelemetry.add("BLUE.OPEN.GATE");
                case "BLUE.GATE.INTAKE":
                    actionList.add(autoActions.blueGateIntakeAction());
                    autoCommendTelemetry.add("BLUE.GATE.INTAKE");
                    break;
                case "BLUE.LOAD.INTAKE":
                    actionList.add(autoActions.blueLoadingZoneIntakeAction());
                    autoCommendTelemetry.add("BLUE.LOAD.INTAKE");
                    break;
                case "UPDATE.POSE":
                    actionList.add(autoActions.updatePoseFromVisionAction(false));
                    autoCommendTelemetry.add("UPDATE.POSE");
                    break;
                case "FORCED.UPDATE.POSE":
                    actionList.add(autoActions.updatePoseFromVisionAction(true));
                    autoCommendTelemetry.add("UPDATE.POSE.FORCED");
                    break;
                case "DELAY.HUNDRED.MS":
                    actionList.add(new SleepAction(0.1));
                    autoCommendTelemetry.add("DELAY.HUNDRED.MS");
                    break;
                case "DELAY.ONE.S":
                    actionList.add(new SleepAction(1));
                    autoCommendTelemetry.add("DELAY.ONE.S");
                    break;
                default:
                    if (action.startsWith("GO.TO.POSE2D")) {
                        String inside = action.substring(
                                action.indexOf("(") + 1,
                                action.indexOf(")")
                        );

                        String[] nums = inside.split(",");
                        double x = Double.parseDouble(nums[0].trim());
                        double y = Double.parseDouble(nums[1].trim());
                        double heading = Double.parseDouble(nums[2].trim());
                        actionList.add(autoActions.goToPoseAction(x, y, heading));
                        autoCommendTelemetry.add("GO.TO.POSE2D(" + x + ", "+ y + ", "+ heading +")");
                    } else {
                        autoCommendTelemetry.add("I read: " + action);
                    }
                    break;
            }
        }
    }

    public Pose2d getStartingPos() {
        return startingPos;
    }

    public RobotContainer.Alliance getAlliance() {
        return alliance;
    }

    public String getStartingPositionTelemetry() {
        return startingPositionTelemetry;
    }

    public ArrayList<Action> getActionList() {
        return actionList;
    }

    public ArrayList<String> getAutoCommendTelemetry() {
        return autoCommendTelemetry;
    }
}
