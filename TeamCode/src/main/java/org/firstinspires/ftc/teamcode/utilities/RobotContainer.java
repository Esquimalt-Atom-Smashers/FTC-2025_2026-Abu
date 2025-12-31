package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.BallColour;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.Direction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.DriveSubsystemState;


/**
 * RobotContainer
 *
 * Central coordinator for all robot subsystems.
 * Provides high-level robot actions and lifecycle control.
 */
public class RobotContainer {

    // Subsystems
    public DriveSubsystem drivebase = null;
    public final ShooterSubsystem shooter;
    public final IntakeTransferSubsystem intake;
    public final VisionSubsystem vision;
    public final ReturnToBaseSubsystem returnToBase;

    private OpMode opMode;
    private Pose2d robotPose;
    private ElapsedTime telemetryTimer;

    /**
     * Represents the alliance color.
     * Used for automatically selecting shooting behavior.
     */
    public enum Alliance {
        RED,
        BLUE
    }
    public Alliance alliance;
    public static final Pose2d RED_GOAL_POSE = new Pose2d(-60, 65 ,0);
    public static final Pose2d BLUE_GOAL_POSE = new Pose2d(-60, -65, 0);
    private final double POSITIONAL_TOLARANCE = 1.0;

    /**
     * Constructor for RobotContainer.
     *
     * All subsystems are created and managed here.
     */
    public RobotContainer(OpMode opMode, Pose2d robotPose, Alliance alliance, DriveSubsystemState driveState, ShooterSubsystem.ShooterState shooterState, IntakeTransferSubsystem.IntakeTransferState intakeTransferState, VisionSubsystem.VisionState visionState) {
        this.opMode = opMode;
        drivebase = new DriveSubsystem(opMode, robotPose, driveState);
        shooter = new ShooterSubsystem(opMode, alliance, shooterState, robotPose);
        intake = new IntakeTransferSubsystem(opMode, intakeTransferState);
        vision = new VisionSubsystem(opMode, alliance, visionState);
        returnToBase = new ReturnToBaseSubsystem();
        shooter.shutDownSubsystem();

        telemetryTimer = new ElapsedTime();
        this.alliance = alliance;
    }

    /**
     * Called periodically during OpMode loop.
     *
     * This method should be lightweight and safe to run every cycle.
     */
    public void runRobot() {
        drivebase.periodic();
        shooter.periodic();
        intake.periodic();
        vision.periodic();
        // TODO: Update subsystems if needed
        if (telemetryTimer.seconds() >= 1.0) {
            opMode.telemetry.clearAll();
            drivebase.addSubsystemTelemetry();
            shooter.addSubsystemTelemetry();
            intake.addSubsystemTelemetry();
            vision.addSubsystemTelemetry();
            opMode.telemetry.update();
            telemetryTimer.reset();
        }
    }

    public void shutDownRobot() {
        drivebase.shutDownSubsystem();
        shooter.shutDownSubsystem();
        intake.shutDownSubsystem();
        vision.shutDownSubsystem();
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
    public void updatePoseFromVision(boolean forceReset) {
        Pose2d visionPose = vision.getLimelightPos();
        Pose2d drivePose = getPose();
        if (visionPose != null) {
            if ((Math.abs(visionPose.position.x - drivePose.position.x) >= POSITIONAL_TOLARANCE || Math.abs(visionPose.position.y - drivePose.position.y) >= POSITIONAL_TOLARANCE) && !forceReset) {
                drivebase.setPose(visionPose);//TODO create a filter to manage a "robot Pose" that takes values from vision and controls how it weights it before updating drive
            }
        }
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
//        shooter.shoot(hoodAngle, rpm);// after ff apply
        intake.feedShooter();
    }

    /**
     * High-level intake command.
     *
     * @param ballColour Ball color to intake
     * @param numberOfBalls Number of balls to intake
     * @param direction Intake direction
     */
    public void intakeBalls(
            BallColour ballColour,
            int numberOfBalls,
            Direction direction
    ) {
        intake.intake(ballColour, numberOfBalls, direction);
    }

    public void drive(double drive, double strafe, double turn) {
        drivebase.driveFieldCentric(drive, strafe, turn);
    }
}
