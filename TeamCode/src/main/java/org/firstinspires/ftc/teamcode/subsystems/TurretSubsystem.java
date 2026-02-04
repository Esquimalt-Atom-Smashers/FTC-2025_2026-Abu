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
    public static class Params {
        public double turretP = 0.01;
        public double turretI = 0.0;
        public double turretD = 0.0;
        public double testSteps = 1;
        public double DEAD_ZONE = 0.005;
        public double SERVO_TO_TURRET_GEAR_RATIO = 2;
        public double kHeadingComp = 1;
        public double MIN_ANGLE_ON_BOT = -340.0;
        public double MAX_ANGLE_ON_BOT = 250.0;
    }
    public static Params PARAMS = new Params();

    private final OpMode opMode;
    private final RobotContainer.Alliance alliance;
    private final Pose2d goalPos;
    private String aimbotLine;
    private boolean isTelemetryEnabled = true;
    // ================= CONFIG =================
// in Turret subsystem
    public RTPAxon.Params TURRET_PARAMS = new RTPAxon.Params();
    {
        TURRET_PARAMS.P = PARAMS.turretP;
        TURRET_PARAMS.I = PARAMS.turretI;
        TURRET_PARAMS.D = PARAMS.turretD;
        TURRET_PARAMS.testSteps = PARAMS.testSteps;
        TURRET_PARAMS.DEAD_ZONE = PARAMS.DEAD_ZONE;
    }

    // Encoder calibration
    private static double SERVO_TO_TURRET_GEAR_RATIO = PARAMS.SERVO_TO_TURRET_GEAR_RATIO; // YOU must calibrate this
    private static final double ENCODER_ZERO_OFFSET_DEG = 0.0; // absolute encoder zero align

    // Feedforward gain for rotation compensation
    private double kHeadingComp = PARAMS.kHeadingComp; // tune if drivetrain model is bad

    // ================= HARDWARE =================
    private RTPAxon turretServoManager;// closed-loop controller
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private RTPAxon.Direction direction = RTPAxon.Direction.FORWARD;

    // ================= STATE =================
    private Pose2d robotPose;
    private double lastFieldAngle = Double.NaN;

    // ================= Subsystem State =================
    public enum TurretState {
        IDLE,
        ROBOT_FRAME_LOCK,   // angle on robot
        GOAL_LOCK    // lock to field coordinate
    }

    private TurretState currentState;

    // Targets
    private double targetRobotFrameDeg = 0.0;
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
    // ================= MODE SETTERS =================

    // 1) Robot-frame angle lock
    public void lockRobotFrame(double angleDeg) {
        currentState = TurretState.ROBOT_FRAME_LOCK;
        targetRobotFrameDeg = findBestAngle(angleDeg);
    }

        // 2) Field-point lock
    public void lockFieldPoint(){
        currentState = TurretState.GOAL_LOCK;
    }

    public void idle(){
        currentState = TurretState.IDLE;
    }

    // ================= CONTROL =================
    private void applyRobotFrameControl(double targetRobotFrameDeg){
        // --- Position target ---
        turretServoManager.setTargetRotation(targetRobotFrameDeg * SERVO_TO_TURRET_GEAR_RATIO);
    }

    // ================= GEOMETRY =================

    private double computeFieldPointAngle(){
        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;
        // Field frame angle
        double fieldAngle = Math.atan2(dy, dx);
        double lastLoopFieldAngle = lastFieldAngle;
        lastFieldAngle = fieldAngle;
        // Convert to robot frame
        double robotFrameAngle = fieldAngle - robotPose.heading.toDouble();
        aimbotLine = "goal target heading: " + Math.toDegrees(fieldAngle);
        if (Double.isNaN(lastLoopFieldAngle)) {
            return findBestAngle(Math.toDegrees(robotFrameAngle));
        } else {
            return findBestAngle(Math.toDegrees(robotFrameAngle + (fieldAngle - lastLoopFieldAngle) * -kHeadingComp));
        }
    }

    private double findBestAngle(double angleDeg) {
        double current = getTurretAngleOnBot(); // continuous, not normalized
        double base = angleDeg; // desired robot-frame angle in degrees

        double best = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;

        // search multiple wraps (enough to cover limits)
        for (int k = -2; k <= 2; k++) {
            double candidate = base + k * 360.0;

            // respect hard limits
            if (candidate < PARAMS.MIN_ANGLE_ON_BOT || candidate > PARAMS.MAX_ANGLE_ON_BOT) {
                continue;
            }

            double error = Math.abs(candidate - current);
            if (error < bestError) {
                bestError = error;
                best = candidate;
            }
        }

        // if nothing valid found, clamp to nearest limit
        if (Double.isNaN(best)) {
            best = Range.clip(base, PARAMS.MIN_ANGLE_ON_BOT, PARAMS.MAX_ANGLE_ON_BOT);
        }
        return best;
    }

    // ================= SENSORS =================

    public double getTurretAngleOnBot(){
        // absolute encoder -> degrees on robot
        double rot = turretServoManager.getTotalRotation() / SERVO_TO_TURRET_GEAR_RATIO;
        return rot + ENCODER_ZERO_OFFSET_DEG;
    }

    // ================= UTIL =================
    private double clipAndNormalize(double deg){
        deg = normalizeDeg(deg);
        return Range.clip(deg, PARAMS.MIN_ANGLE_ON_BOT, PARAMS.MAX_ANGLE_ON_BOT);
    }

    private double normalizeDeg(double deg){
        while(deg > 180) deg -= 360;
        while(deg < -180) deg += 360;
        return deg;
    }

    // ================= TUNING =================
    public double setTargetRobotFrameDeg () {
        return targetRobotFrameDeg;
    }
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
                applyRobotFrameControl(targetRobotFrameDeg);
                break;
            case GOAL_LOCK:
                targetDeg = computeFieldPointAngle();
                lastFieldAngle = targetDeg;
                applyRobotFrameControl(targetDeg);
                break;
            default:
                return;
        }
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
            opMode.telemetry.addLine(aimbotLine);
            opMode.telemetry.addLine(turretServoManager.log());
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

