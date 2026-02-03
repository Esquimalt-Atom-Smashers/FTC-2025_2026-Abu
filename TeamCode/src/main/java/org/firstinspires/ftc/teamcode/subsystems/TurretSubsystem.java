package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opensource.FTC.RTPAxon.RTPAxon;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
@Config
public class TurretSubsystem implements SubsystemBase{
    private final OpMode opMode;
    private final RobotContainer.Alliance alliance;
    private final Pose2d goalPos;
    private boolean isTelemetryEnabled = true;
    // ================= CONFIG =================
    public RTPAxon.Params TURRET_PARAMS = new RTPAxon.Params(){
        public double P = 0.0;
        public double I = 0.0;
        public double D = 0.0;
        public double testSteps = 1;
        public double DEAD_ZONE = 0.005;
    };
    private static final double MIN_ANGLE_ON_BOT = -180.0;
    private static final double MAX_ANGLE_ON_BOT =  180.0;

    // Encoder calibration
    private static final double DEGREE_PER_ROTATION = 180; // YOU must calibrate this
    private static final double ENCODER_ZERO_OFFSET_DEG = 0.0; // absolute encoder zero align

    // Feedforward gain for rotation compensation
    private double kHeadingComp = 1.0; // tune if drivetrain model is bad

    // ================= HARDWARE =================
    private RTPAxon turretServoManager;// closed-loop controller
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private RTPAxon.Direction direction = RTPAxon.Direction.FORWARD;

    // ================= STATE =================
    private Pose2d robotPose;
    private double robotHeadingRate = 0.0; // rad/s

    // ================= Subsystem State =================
    public enum TurretState {
        IDLE,
        ROBOT_FRAME_LOCK,   // angle on robot
        GOAL_LOCK    // lock to field coordinate
    }

    private TurretState currentState = TurretState.IDLE;

    // Targets
    private double targetRobotFrameDeg = 0.0;
    private double targetX = 0.0;
    private double targetY = 0.0;


    public TurretSubsystem(OpMode opMode, RobotContainer.Alliance alliance, TurretState turretState, Pose2d pose2d, Pose2d goalPos) {
        this.opMode = opMode;
        this.alliance = alliance;
        turretServo = opMode.hardwareMap.get(CRServo.class, "turretServo");
        turretEncoder = opMode.hardwareMap.get(AnalogInput.class, "turretEncoder");
        turretServoManager = new RTPAxon(turretServo, turretEncoder, direction, TURRET_PARAMS);
        this.currentState = turretState;
        robotPose = pose2d;
        this.goalPos = goalPos;
    }


    // ================= INPUT UPDATES =================
    public void updateRobotPose(Pose2d pose){
        this.robotPose = pose;
    }

    public void updateHeadingRate(double radPerSec){
        this.robotHeadingRate = radPerSec;
    }

    // ================= MODE SETTERS =================

    // 1) Robot-frame angle lock
    public void lockRobotFrame(double angleDeg){
        currentState = TurretState.ROBOT_FRAME_LOCK;
        targetRobotFrameDeg = clipAndNormalize(angleDeg);
    }

    // 2) Field-point lock
    public void lockFieldPoint(double x, double y){
        currentState = TurretState.GOAL_LOCK;
        targetX = x;
        targetY = y;
    }

    public void idle(){
        currentState = TurretState.IDLE;
    }

    // ================= CONTROL =================

    private void applyCompensatedControl(double targetRobotFrameDeg){
        // --- Position target ---
        turretServoManager.setTargetRotation(degToRot(targetRobotFrameDeg + -kHeadingComp * Math.toDegrees(robotHeadingRate)));
    }

    // ================= GEOMETRY =================

    private double computeFieldPointAngle(){
        double dx = targetX - robotPose.position.x;
        double dy = targetY - robotPose.position.y;

        // Field frame angle
        double fieldAngle = Math.atan2(dy, dx);

        // Convert to robot frame
        double robotFrameAngle = fieldAngle - robotPose.heading.toDouble();

        return clipAndNormalize(Math.toDegrees(robotFrameAngle));
    }

    // ================= SENSORS =================

    public double getTurretAngleOnBot(){
        // absolute encoder -> degrees on robot
        double rot = turretServoManager.getTotalRotation();
        return rotToDeg(rot) + ENCODER_ZERO_OFFSET_DEG;
    }

    // ================= UTIL =================

    private double rotToDeg(double rot){
        return rot / DEGREE_PER_ROTATION;
    }

    private double degToRot(double deg){
        return deg * DEGREE_PER_ROTATION;
    }

    private double clipAndNormalize(double deg){
        deg = normalizeDeg(deg);
        return Range.clip(deg, MIN_ANGLE_ON_BOT, MAX_ANGLE_ON_BOT);
    }

    private double normalizeDeg(double deg){
        while(deg > 180) deg -= 360;
        while(deg < -180) deg += 360;
        return deg;
    }

    // ================= TUNING =================

    public void setHeadingCompGain(double k){
        this.kHeadingComp = k;
    }
    //
    /**
     * gets called from robot container every loop
     */
    @Override
    public void periodic() {
        if(currentState == TurretState.IDLE){
            turretServoManager.setPower(0);
            return;
        }
        double targetDeg;
        switch(currentState){
            case ROBOT_FRAME_LOCK:
                targetDeg = targetRobotFrameDeg;
                break;
            case GOAL_LOCK:
                targetDeg = computeFieldPointAngle();
                break;
            default:
                return;
        }
        // Apply feedforward compensation
        applyCompensatedControl(targetDeg);
        turretServoManager.update();
    }

    /**
     * Enables or disables telemetry output for this subsystem. defaults to true
     *
     * @param enabled True to enable telemetry, false to disable
     */
    @Override
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (isTelemetryEnabled) {
            opMode.telemetry.addData("Current TA On Bot", getTurretAngleOnBot());
            opMode.telemetry.addData("Target TA On Bot", targetRobotFrameDeg);
        }
    }

    /**
     * Resets the subsystem to a known safe state.
     */
    @Override
    public void resetSubsystem() {
    }

    /**
     * Safely shuts down the subsystem. Motors should stop
     */
    @Override
    public void shutDownSubsystem() {
        idle();
    }

    /**
     * Returns the current state of the subsystem.
     *
     * @return The current subsystem state (subsystem-specific enum)
     */
    @Override
    public Enum<?> getState() {
        return currentState;
    }
    public void setCurrentState(TurretState turretState) {currentState = turretState;}

}

