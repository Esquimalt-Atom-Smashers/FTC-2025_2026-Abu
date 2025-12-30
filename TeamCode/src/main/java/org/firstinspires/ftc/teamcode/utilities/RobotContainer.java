package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.BallColour;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.Direction;


/**
 * RobotContainer
 *
 * Central coordinator for all robot subsystems.
 * Provides high-level robot actions and lifecycle control.
 */
public class RobotContainer {

    // Subsystems
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeTransferSubsystem intake;
    private final VisionSubsystem vision;
    private final ReturnToBaseSubsystem returnToBase;

    /**
     * Constructor for RobotContainer.
     *
     * All subsystems are created and managed here.
     */
    public RobotContainer() {
        drive = new DriveSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeTransferSubsystem();
        vision = new VisionSubsystem();
        returnToBase = new ReturnToBaseSubsystem();
    }

    /**
     * Called periodically during OpMode loop.
     *
     * This method should be lightweight and safe to run every cycle.
     */
    public void runRobot() {
        // TODO: Update subsystems if needed
        // Example: drive.periodic(), shooter.periodic()
    }

    /**
     * Returns the robot's current pose. combining multiple sensors
     *
     * @return Current estimated pose
     */
    public Pose2d getPose() {
        return drive.getPose();
    }//TODO add more code to integrate limelight

    /**
     * Updates the robot pose using vision data.
     *
     * Vision data is fused with drive localization.
     */
    public void updatePoseFromVision() {
        Pose2d visionPose = vision.getLimelightPos(getPose().heading.toDouble());

        if (visionPose != null) {
            drive.setPose(visionPose);//TODO create a filter to manage a "robot Pose" that takes values from vision and controls how it weights it before updating drive
        }
    }

    /**
     * Commands the robot to drive to a target pose.
     *
     * @param pose Target pose
     */
    public void goToPose(Pose2d pose) {
        drive.goToPose(pose);
    }

    /**
     * High-level shooting command.
     *
     * @param ballColour The ball color being shot
     * @param hoodAngle Shooter hood angle
     * @param rpm Shooter flywheel speed
     */
    public void shoot(BallColour ballColour, double hoodAngle, double rpm) {
        shooter.shoot(hoodAngle, rpm);
        intake.feedShooter(ballColour);
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
}
