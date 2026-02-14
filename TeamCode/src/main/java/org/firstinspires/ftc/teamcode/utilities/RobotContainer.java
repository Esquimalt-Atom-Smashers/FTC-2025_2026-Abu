package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_GOAL_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_GOAL_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_GOAL_Y;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.DriveSubsystemState;


/**
 * RobotContainer
 *
 * Central coordinator for all robot subsystems.
 * Provides high-level robot actions and lifecycle control.
 */
public class RobotContainer {

    // Subsystems
    public DriveSubsystem drivebase;
    public final TwoMotorShooterSubsystem shooter;
    public final IntakeTransferSubsystem intake;
    public final TurretSubsystem turretSubsystem;
//    public final VisionSubsystem vision;
    public final ReturnToBaseSubsystem returnToBase;
//    public final LEDSubsystem ledSubsystem;

    private OpMode opMode;
    private Pose2d robotPose;
    private String opModeTelemetry;
    public Pose2d goalPos;
    private ElapsedTime telemetryTimer;
    private boolean isAuto = false;
    private boolean firstRun = true;
    private ElapsedTime autoTimer;
    /**
     * Represents the alliance color.
     * Used for automatically selecting shooting behavior.
     */
    public enum Alliance {
        RED,
        BLUE
    }
    public Alliance alliance;
    private final double POSITIONAL_TOLARANCE = 1.0;

    /**
     * Constructor for RobotContainer.
     *
     * All subsystems are created and managed here.
     */
    public RobotContainer(OpMode opMode, Pose2d robotPose, Alliance alliance, DriveSubsystemState driveState, TwoMotorShooterSubsystem.ShooterState shooterState, TurretSubsystem.TurretState turretState,IntakeTransferSubsystem.IntakeTransferState intakeTransferState, VisionSubsystem.VisionState visionState) {
        goalPos = alliance == RobotContainer.Alliance.RED? new Pose2d(RED_GOAL_X, RED_GOAL_Y, Math.toRadians(RED_GOAL_HEADING)): new Pose2d(BLUE_GOAL_X, BLUE_GOAL_Y, Math.toRadians(BLUE_GOAL_HEADING));

        this.opMode = opMode;
        drivebase = new DriveSubsystem(opMode, alliance, driveState, robotPose, goalPos);
        shooter = new TwoMotorShooterSubsystem(opMode, alliance, shooterState, robotPose);
        turretSubsystem = new TurretSubsystem(opMode, alliance, turretState, robotPose, goalPos);
        intake = new IntakeTransferSubsystem(opMode, intakeTransferState);
//        vision = new VisionSubsystem(opMode, alliance, visionState, robotPose);
        returnToBase = new ReturnToBaseSubsystem();
//        ledSubsystem = new LEDSubsystem(opMode, alliance);
        shooter.shutDownSubsystem();

        telemetryTimer = new ElapsedTime();
        this.alliance = alliance;
        this.robotPose = robotPose;

        if (driveState == DriveSubsystemState.AUTO) {
            isAuto = true;
            autoTimer = new ElapsedTime();
            autoTimer.reset();
        }
    }

    /**
     * Called periodically during OpMode loop.
     *
     * This method should be lightweight and safe to run every cycle.
     */
    public void runRobot() {
        if (firstRun) {
            firstRun = false;
//            ledSubsystem.normalLight();
        }
        robotPose = drivebase.getPose();
        drivebase.periodic();
        shooter.periodic();
        turretSubsystem.updateRobotPose(robotPose, drivebase.getMecanumDrive().localizer.update());
        turretSubsystem.periodic();
        intake.periodic();
//        vision.updateCurrentPose(drivebase.getPose());
//        vision.periodic();
        // TODO: Update subsystems if needed
        if (telemetryTimer.seconds() >= Property.TELEMETRY_UPDATE_TIME) {
            opMode.telemetry.clearAll();
            drivebase.addSubsystemTelemetry();
            shooter.addSubsystemTelemetry();
            turretSubsystem.addSubsystemTelemetry();
            intake.addSubsystemTelemetry();
//            vision.addSubsystemTelemetry();
            opMode.telemetry.addLine(opModeTelemetry);
            opMode.telemetry.update();
            telemetryTimer.reset();
        }
    }

    public void addOpModeTelemetry(String line) {
        opModeTelemetry = line;
    }

    public void updatePositionHolderAuto() {
        Pose2d lastPose = getPose();
        RobotPositionHolder.storePos(lastPose.position.x, lastPose.position.y, lastPose.heading.toDouble());
        if (isAuto && autoTimer.seconds() >= 29.5) {
        }
    }
    public void shutDownRobot() {
        drivebase.shutDownSubsystem();
        shooter.shutDownSubsystem();
        intake.shutDownSubsystem();
//        vision.shutDownSubsystem();
//        ledSubsystem.shutDownSubsystem();
    }

    /**
     * Returns the robot's current pose. combining multiple sensors
     *
     * @return Current estimated pose
     */
    public Pose2d getPose() {
        return drivebase.getPose();
    }//TODO add more code to integrate limelight

    /**
     * Updates the robot pose using vision data.
     *
     * Vision data is fused with drive localization.
     */
    public boolean updatePoseFromVision(boolean forceReset) {
        Pose2d drivePose = getPose();
//        vision.updateCurrentPose(drivePose);
//        vision.periodic();
//        Pose2d visionPose = vision.getLimelightPosMT2();
//        if (visionPose != null) {
//            if ((Math.abs(visionPose.position.x - drivePose.position.x) <= POSITIONAL_TOLARANCE && Math.abs(visionPose.position.y - drivePose.position.y) <= POSITIONAL_TOLARANCE) || !forceReset) {
//                drivebase.setPose(visionPose);//TODO create a filter to manage a "robot Pose" that takes values from vision and controls how it weights it before updating drive
//                return true;
//            }
//        }
        //did not update pose
        return false;
    }

    /**
     * Commands the robot to drive to a target pose.
     *
     * @param pose Target pose
     */
    public void goToPose(Pose2d pose) {
        drivebase.goToPose(pose);
    }

    /**High-level shooting command.*/
    public void shoot() {
        TwoMotorShooterSubsystem.FlywheelSetting flywheelSetting = shooter.distanceToFlywheelSetting(drivebase.getDistanceToGoal());
        shooter.shoot(flywheelSetting);
    }

    public void drive(double drive, double strafe, double turn) {
        drivebase.driveFieldCentric(drive, strafe, turn);
//        ledSubsystem.normalLight();
    }

    public class AutoRunRobot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runRobot();
            updatePositionHolderAuto();
            shoot();
            return true;
        }
    }

    public Action autoRunRobot() {return new AutoRunRobot();}
}
