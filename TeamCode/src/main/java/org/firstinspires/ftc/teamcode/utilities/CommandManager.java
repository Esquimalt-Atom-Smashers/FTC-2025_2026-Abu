package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
@Config
public class CommandManager {
    private DriveSubsystem driveSubsystem;
    private IntakeFeedSubsystem intakeFeedSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private LimelightSybsystem limelightSybsystem;

    private boolean ISREDALLIANCE;

    //aimbot
    public static class Params {
        public double P = 0.5;
        public double I = 0.0;
        public double D = 0.0;
        public double F = 0.0;
        public double headingError = 0.0;
    }
    public static Params PARAMS = new Params();
    private PIDFController turnController = new PIDFController(PARAMS.P, PARAMS.I, PARAMS.D, PARAMS.F);
    private double ANGULAR_TOLERANCE = 1.0;

    public CommandManager(DriveSubsystem driveSubsystem, IntakeFeedSubsystem intakeFeedSubsystem, FlywheelSubsystem flywheelSubsystem, LimelightSybsystem limelightSybsystem, boolean isRedAlliance) {
        this.driveSubsystem = driveSubsystem;
        this.intakeFeedSubsystem = intakeFeedSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.limelightSybsystem = limelightSybsystem;
        this.ISREDALLIANCE = isRedAlliance;
    }

    public void aimbotAssistedDrive(double drive, double strafe, boolean forceReset) {
        driveSubsystem.getMecanumDrive().localizer.setPose(limelightSybsystem.getBotPose2D(driveSubsystem.getCurrentPos(), forceReset));

        Pose2d goalPos = limelightSybsystem.getIsRedAlliance()? limelightSybsystem.RED_GOAL_POSE: limelightSybsystem.BLUE_GOAL_POSE;
        double goalPosX = goalPos.position.x;
        double goalPosY = goalPos.position.y;
        double robotPosX = driveSubsystem.getCurrentPos().position.x;
        double robotPosY = driveSubsystem.getCurrentPos().position.y;

        double dX = Math.abs(goalPosX - robotPosX);
        double dY = Math.abs(goalPosY - robotPosY);
        double hyp = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
        double targetHeading;
        if (limelightSybsystem.getIsRedAlliance()) {
            targetHeading = Math.toRadians(180) - Math.acos(dX / hyp) - Math.toRadians(90);
//            if (Math.toDegrees(targetHeading) < -180) { targetHeading += Math.toRadians(360);}
//            if (Math.toDegrees(targetHeading) >= 180) { targetHeading -= Math.toRadians(360);}
        } else {
            double headingError = PARAMS.headingError;
            targetHeading = (Math.toRadians(180) + (Math.toRadians(90) - Math.asin(dX/hyp)) - Math.toRadians(90)) + Math.toRadians(headingError);
            if (Math.toDegrees(targetHeading) < -180) { targetHeading += Math.toRadians(360);}
            if (Math.toDegrees(targetHeading) >= 180) { targetHeading -= Math.toRadians(360);}
        }

        driveSubsystem.opMode.telemetry.addData("goal target heading", Math.toDegrees(targetHeading));
//        driveSubsystem.opMode.telemetry.addData("current heading", Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()));
//        driveSubsystem.opMode.telemetry.addData("heading error",driveSubsystem.getCurrentPos().heading.toDouble() - targetHeading);
//        driveSubsystem.opMode.telemetry.addData("is within tolerance", Math.abs(Math.toDegrees(targetHeading) - Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble())) <= ANGULAR_TOLERANCE);

        double turn;
        if (Math.abs(Math.toDegrees(targetHeading) - Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble())) >= ANGULAR_TOLERANCE) {
            turn = turnController.calculate(driveSubsystem.getCurrentPos().heading.toDouble(), targetHeading);
        } else {
            turn = 0.0;
        }
//        driveSubsystem.opMode.telemetry.addData("turn power", turn);
        driveSubsystem.drive(drive, strafe, turn);
    }

    public double getDistanceToGoal() {
        Pose2d currentPos = driveSubsystem.getCurrentPos();
        double driveDistanceEstimate;
        Pose2d goalPos;
//        if (ISREDALLIANCE) driveDistanceEstimate = Math.sqrt(Math.pow(currentPos.position.x, 2) + Math.pow(currentPos.position.y, 2));
//        else driveDistanceEstimate = Math.sqrt(Math.pow(currentPos.position.x, 2) + Math.pow(currentPos.position.y, 2));
        if (ISREDALLIANCE) {
            goalPos = limelightSybsystem.RED_GOAL_POSE;
        } else {
            goalPos = limelightSybsystem.BLUE_GOAL_POSE;
        }
        driveDistanceEstimate = Math.sqrt(Math.pow(currentPos.position.x - goalPos.position.x, 2) + Math.pow(currentPos.position.y - goalPos.position.y, 2));
        if (limelightSybsystem.getDistanceFromGoal() != 0) return limelightSybsystem.getDistanceFromGoal();
        return driveDistanceEstimate;
    }
    public class ShootArtifactAction implements Action {
//        TODO: adjust numbers from testing
        private final double RPM_TOLERANCE = flywheelSubsystem.RPM_TOLERANCE;
        private final double SHOOTING_RPM_DROP = -1.0;

        public double targetVelocity;
        public boolean isAccelerated;
        public ShootArtifactAction(double targetRPM, double maxPower) {

            flywheelSubsystem.setFlywheelTargetVelocity(flywheelSubsystem.rpmToTps(targetRPM));
            flywheelSubsystem.setFlywheelMaxPower(maxPower);

            intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.REVERSED);
            isAccelerated = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelSubsystem.runFlywheelControl();
            if (!isAccelerated && Math.abs(flywheelSubsystem.getFlywheelRPM() - targetVelocity) < RPM_TOLERANCE) {
                isAccelerated = true;
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.SPINNING);
            }
            if (isAccelerated && Math.abs(flywheelSubsystem.getFlywheelRPM() - targetVelocity) > SHOOTING_RPM_DROP) {
                return false;
            } else {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.REVERSED);
                return true;
            }
        }
    }

    public Action shootArtifactAction(double targetRPM, double maxPower) {return new ShootArtifactAction(targetRPM, maxPower);}

    public class Periodic implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            driveSubsystem.periodic();
            flywheelSubsystem.runFlywheelControl();
            limelightSybsystem.periodic();

//            driveSubsystem.opMode.telemetry.addData("current heading", driveSubsystem.getHeading());
//            driveSubsystem.opMode.telemetry.addData("heading error", driveSubsystem.driveHeadingError);
//            driveSubsystem.opMode.telemetry.addData("ty", limelightSybsystem.getTy());
//            driveSubsystem.opMode.telemetry.addData("turning heading", limelightSybsystem.getTy() + Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()));
//            driveSubsystem.opMode.telemetry.addData("targetRPM", flywheelSubsystem.getTargetVelocity());
//            driveSubsystem.opMode.telemetry.addData("currentRPM", flywheelSubsystem.getFlywheelRPM());
//            driveSubsystem.opMode.telemetry.update();
            return false;
        }
    }

    public Action periodic() {return new Periodic();}
}
