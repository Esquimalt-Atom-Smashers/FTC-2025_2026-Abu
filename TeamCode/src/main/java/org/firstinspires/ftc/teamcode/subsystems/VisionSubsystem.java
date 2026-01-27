package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer.*;

import java.util.ArrayList;
import java.util.List;

/**
 * VisionSubsystem
 *
 * Handles vision processing and targeting using a vision system
 * (e.g., Limelight, OpenCV, AprilTags).
 *
 * This class currently contains only structure and documentation.
 */
@Config
public class VisionSubsystem implements SubsystemBase{
    public static class Params {
        public double redHeadingOffset = 0.0;
        public double blueHeadingOffset = 0.0;
    }
    public static Params PARAMS = new Params();
    /**
     * Represents possible detected field patterns.
     */
    public enum Pattern {
        PATTERN1,
        PATTERN2,
        PATTERN3
    }

    public enum VisionState{
        DISABLED,
        TRACKING_GOAL
    }
    private Limelight3A limelight3A;
    private String LIMELIGHT_NAME = "limelight";

    private final int RED_GOAL_APRILTAG_PIPELINE = 0;
    private final int BLUE_GOAL_APRILTAG_PIPELINE = 1;
    private final int PULL_RATE_HZ = 10;

    public final Pose2d RED_GOAL_POSE = new Pose2d(-60, 65 ,0);
    public final Pose2d BLUE_GOAL_POSE = new Pose2d(-60, -65, 0);
    private double METER_TO_INCH = 39.37008;
    private final double POSITIONAL_TOLARANCE = 1.0;

    private Alliance alliance;
    private OpMode opMode;
    private VisionState currentState;

    private Pose3D botPoseMT2;
    private Pose3D botPoseMT1;
    private double ty = -9999.9999;
    private double tx = -9999.9999;
    private double ta = -9999.9999;
    private int tagId = 0;
    private Pose2d pose2d;
    private boolean LLGotData = false;
    boolean isTelemetryEnabled = true;


    /**
     * Constructor for the VisionSubsystem.
     *
     * Vision hardware and pipelines should be
     * initialized here later.
     */
    public VisionSubsystem(OpMode opMode, Alliance alliance, VisionState visionState, Pose2d pose) {
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight3A.setPollRateHz(PULL_RATE_HZ);
        this.alliance = alliance;
        this.opMode = opMode;
        currentState = visionState;
        if (alliance == Alliance.RED) {
            limelight3A.pipelineSwitch(RED_GOAL_APRILTAG_PIPELINE);
        } else {
            limelight3A.pipelineSwitch(BLUE_GOAL_APRILTAG_PIPELINE);
        }
        limelight3A.start();
        updateCurrentPose(pose);
    }

    /** Shall be run every loop **/
    public void updateCurrentPose(Pose2d pose2d) {
        this.pose2d = pose2d;
        limelight3A.updateRobotOrientation(Math.toDegrees(pose2d.heading.toDouble()));
    }

    public double getTy() {
        return ty;
    }
    public double getTx() {
        return tx;
    }
    public double getTa() {
        return ta;
    }

    /**
     * Returns the robot's position estimated by the Limelight.
     ** @return The estimated pose from vision
     */
    public Pose2d getLimelightPosMT1() {
        limelight3A.updateRobotOrientation(alliance == Alliance.RED? Math.toDegrees(pose2d.heading.toDouble()) + PARAMS.redHeadingOffset : Math.toDegrees(pose2d.heading.toDouble()) + PARAMS.blueHeadingOffset);
        if (botPoseMT1 == null) return null;
        Pose2d returningPose = new Pose2d(botPoseMT1.getPosition().x * METER_TO_INCH, botPoseMT1.getPosition().y * METER_TO_INCH, botPoseMT1.getOrientation().getYaw(AngleUnit.RADIANS));
        return returningPose;
    }

    /**
     * Returns the robot's position estimated by the Limelight in MT2.
     ** @return The estimated pose from vision
     */
    public Pose2d getLimelightPosMT2() {
        limelight3A.updateRobotOrientation(alliance == Alliance.RED? Math.toDegrees(pose2d.heading.toDouble()) + PARAMS.redHeadingOffset : Math.toDegrees(pose2d.heading.toDouble()) + PARAMS.blueHeadingOffset);
        if (botPoseMT2 == null) return null;
        Pose2d returningPose = new Pose2d(botPoseMT2.getPosition().x * METER_TO_INCH, botPoseMT2.getPosition().y * METER_TO_INCH, botPoseMT2.getOrientation().getYaw(AngleUnit.RADIANS));
        return returningPose;
    }

    /**
     * Returns the heading the robot should turn to
     * in order to face the target for the given alliance.
     *
     * @param alliance The current alliance
     * @return Target heading in radians or degrees (define later)
     */
    public double getTargetHeading(Alliance alliance) {
        // TODO: Calculate target heading from vision data
        return 0.0;
    }

    /**
     * Returns the detected field pattern or target configuration.
     *
     * @return Identifier for the detected pattern
     */
    public Pattern getPattern() {
        // TODO: Determine pattern from vision pipeline
        return null;
    }
//--------------------Common functions across subsystems--------------------
    /**
     * Runs every loop
     */
    public void periodic() {
        if (currentState != VisionState.DISABLED) {
            LLResult result = limelight3A.getLatestResult();
            if (result.isValid()) {
                ty = result.getTy();
                tx = result.getTx();
                ta = result.getTa();
                botPoseMT2 = result.getBotpose_MT2();
                botPoseMT1 = result.getBotpose();
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 21 || fr.getFiducialId() == 22 || fr.getFiducialId() == 33){
                        tagId = fr.getFiducialId();
                    }
                }
                LLGotData = true;
            } else {
                ty = 9999.9999;
                tx = 9999.9999;
                ta = 9999.9999;
                botPoseMT1 = null;
                botPoseMT2 = null;
                tagId = 0;
                LLGotData = false;
            }
        } else {
            pauseSubsystem();
            ty = 9999.9999;
            tx = 9999.9999;
            ta = 9999.9999;
            botPoseMT1 = null;
            botPoseMT2 = null;
            tagId = 0;
            LLGotData = false;
        }
    }

    /**
     * Enables or disables telemetry output for this subsystem.
     *
     * @param enabled True to enable telemetry, false to disable
     */
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (isTelemetryEnabled) {
            opMode.telemetry.addData("LL data", LLGotData);
            Pose2d llPoseMT1 = getLimelightPosMT1();
            if (getLimelightPosMT1() != null) {
                opMode.telemetry.addData("LL PoseMT1", "X: %.2f, Y: %.2f, H: %.2f", llPoseMT1.position.x, llPoseMT1.position.y, Math.toDegrees(llPoseMT1.heading.toDouble()));
            }
            Pose2d llPose = getLimelightPosMT2();
            if (getLimelightPosMT2() != null) {
                opMode.telemetry.addData("LL Pose", "X: %.2f, Y: %.2f, H: %.2f", llPose.position.x, llPose.position.y, Math.toDegrees(llPose.heading.toDouble()));
            }
        }
    }

    /**
     * Resets the subsystem to a known safe state.
     */
    public void resetSubsystem(){
        limelight3A.resetDeviceConfigurationForOpMode();
    }

    /**
     * Safely shuts down the subsystem.
     *
     * Motors should stop and resources should be released.
     */
    public void pauseSubsystem() {
        limelight3A.pause();
    }

    public void shutDownSubsystem() {
        limelight3A.stop();
    }

    public VisionState getState() {
        return currentState;
    }
    public void setCurrentState(VisionState visionState) {currentState = visionState;}
}
