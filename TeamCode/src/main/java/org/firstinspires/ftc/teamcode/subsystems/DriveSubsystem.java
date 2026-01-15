package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

/**
 * DriveSubsystem
 *
 * Handles all robot movement, including field-centric and
 * robot-centric driving, pose tracking, and autonomous navigation.
 *
 * This class currently contains only structure and documentation.
 */
public class DriveSubsystem implements SubsystemBase {

    public OpMode opMode;
    public RobotContainer.Alliance alliance;

    private boolean isFieldCentric;
    private double speedMultiplier = 1.0;

    private MecanumDrive mecanumDrive;
    private Pose2d currentPose;
    public double driveHeadingError = 0.0;
    private boolean isTelemetryEnabled = true;

    public enum DriveSubsystemState{
        DISABLED,
        TELEOP_DRIVING,
        TELEOP_AIMBOT,
        AUTO
    }
    public DriveSubsystemState currentState;

    public static class Params {
        public double P = 0.5;
        public double I = 0.0;
        public double D = 0.0;
        public double F = 0.0;
        public double ANGULAR_TOLERANCE = 1.0;
    }
    public static Params PARAMS = new Params();
    private PIDFController turnController;
    private Pose2d goalPos;
    /**
     * Constructor for the DriveSubsystem.
     * <p>
     * Drive motors, IMU, and localization systems
     * should be initialized here later.
     */
    public DriveSubsystem(OpMode opMode, RobotContainer.Alliance alliance, DriveSubsystemState state, Pose2d startingPose, Pose2d goalPose) {
        this.opMode = opMode;
        this.alliance = alliance;
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, startingPose);
        getMecanumDrive().localizer.setPose(startingPose);
        this.currentPose = startingPose;
        isFieldCentric = true;

        turnController = new PIDFController(PARAMS.P, PARAMS.I, PARAMS.D, PARAMS.F);
        setDriveHeadingError();
        currentState = state;
        this.goalPos = goalPose;
    }

    /**
     * Drives the robot using field-centric control.
     *
     * @param drive  Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn   Rotational movement input
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        double botHeading = getFieldCentricHeading();
        double rotX = strafe * Math.cos(botHeading) - drive * Math.sin(botHeading);
        double rotY = strafe * Math.sin(botHeading) + drive * Math.cos(botHeading);

        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(rotY * speedMultiplier, rotX * speedMultiplier), turn * speedMultiplier
                ));
    }

    /**
     * Drives the robot using robot-centric control.
     *
     * @param drive  Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn   Rotational movement input
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(drive * speedMultiplier, strafe * speedMultiplier), turn * speedMultiplier
                ));
    }

    public void aimbotAssistedDrive(double drive, double strafe, double turn) {
        double goalPosX = goalPos.position.x;
        double goalPosY = goalPos.position.y;
        currentPose = getPose();
        double robotPosX = currentPose.position.x;
        double robotPosY = currentPose.position.y;

        double dX = Math.abs(goalPosX - robotPosX);
        double dY = Math.abs(goalPosY - robotPosY);
        double hyp = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
        double targetHeading;

        if (alliance == RobotContainer.Alliance.RED) {
            targetHeading = Math.toRadians(180) - Math.acos(dX / hyp) - Math.toRadians(90);
        } else {
            targetHeading = (Math.toRadians(180) + (Math.toRadians(90) - Math.asin(dX/hyp)) - Math.toRadians(90));
            if (Math.toDegrees(targetHeading) < -180) { targetHeading += Math.toRadians(360);}
            if (Math.toDegrees(targetHeading) >= 180) { targetHeading -= Math.toRadians(360);}
        }

        opMode.telemetry.addData("goal target heading", Math.toDegrees(targetHeading));
//        driveSubsystem.opMode.telemetry.addData("current heading", Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()));
//        driveSubsystem.opMode.telemetry.addData("heading error",driveSubsystem.getCurrentPos().heading.toDouble() - targetHeading);
//        driveSubsystem.opMode.telemetry.addData("is within tolerance", Math.abs(Math.toDegrees(targetHeading) - Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble())) <= ANGULAR_TOLERANCE);

        double turnSuggested;
        if (Math.abs(Math.toDegrees(targetHeading) - Math.toDegrees(currentPose.heading.toDouble())) >= PARAMS.ANGULAR_TOLERANCE) {
            turnSuggested = turnController.calculate(currentPose.heading.toDouble(), targetHeading);
        } else {
            turnSuggested = 0.0;
        }

        if (turnSuggested >= 0) {
            turn = Range.clip(turn, -1.0, turnSuggested);
        } else {
            turn = Range.clip(turn, turnSuggested, 1.0);
        }
        opMode.telemetry.addData("turn power", turnSuggested);
        driveFieldCentric(drive, strafe, turn);
    }

    public void switchFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void changeSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = Range.clip(speedMultiplier, 0, 1);
    }

    public void setDriveHeadingError() {
        driveHeadingError = mecanumDrive.localizer.getPose().heading.toDouble();
    }

    public void setDriveHeadingErrorTo(double fieldForwardRadians) {
        driveHeadingError = fieldForwardRadians;
    }

    /**
     *@return heading for fieldCentric teleOp driving
     */
    public double getFieldCentricHeading() {
        //get radian
        double headingRadian = -(driveHeadingError - mecanumDrive.localizer.getPose().heading.toDouble());
        if (Math.toDegrees(headingRadian) >= 180) {
            headingRadian -= Math.toRadians(360);
        } else if (Math.toDegrees(headingRadian) < -180) {
            headingRadian += Math.toRadians(360);
        }
        return headingRadian;
    }

    public void setHeading (double degrees) {
        currentPose = mecanumDrive.localizer.getPose();
        mecanumDrive.localizer.setPose(new Pose2d(currentPose.position.x, currentPose.position.y, Math.toRadians(degrees)));
    }
    /**
     * @return The current Pose (position and heading)
     */
    public Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    /**
     * Sets the robot's current pose.
     * @param pose The pose to set as the current position
     */
    public void setPose(Pose2d pose) {
        currentPose = pose;
        mecanumDrive.localizer.setPose(pose);
    }

    /**
     * Commands the robot to navigate to a target pose.
     * @param pose The target pose to drive to
     */
    public void goToPose(Pose2d pose) {
        // TODO: Implement path following or PID control, if we want to.
    }

    public double getDistanceToGoal() {
        getPose();
        double driveDistanceEstimate;
        driveDistanceEstimate = Math.sqrt(Math.pow(currentPose.position.x - goalPos.position.x, 2) + Math.pow(currentPose.position.y - goalPos.position.y, 2));
        return driveDistanceEstimate;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }
//--------------------Common functions across subsystems--------------------
    /** shall be ran every loop*/
    @Override
    public void periodic() {
        if (currentState == DriveSubsystemState.DISABLED) {
            shutDownSubsystem();
        }
    }

    /**
     * Enables or disables telemetry output for this subsystem.
     * @param enabled True to enable telemetry, false to disable
     */
    @Override
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (isTelemetryEnabled) {
            Pose2d pose = getPose();
            opMode.telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.2f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            opMode.telemetry.addData("FC Heading", getFieldCentricHeading());
        }
    }

    /**Resets the subsystem to a known safe state.*/
    @Override
    public void resetSubsystem() {

    }

    /**
     * Safely shuts down the subsystem.
     * Motors should stop and resources should be released.
     */
    @Override
    public void shutDownSubsystem() {
//        mecanumDrive.setDrivePowers(
//                new PoseVelocity2d(
//                        new Vector2d(0, 0), 0
//                )
//        );
    }

    /**
     * Returns the current state of the subsystem.
     *@return The current subsystem state (subsystem-specific enum)
     */
    @Override
    public DriveSubsystemState getState() {
        return currentState;
    }

    public void setState(DriveSubsystemState targetState) {
        currentState = targetState;
    }

}
