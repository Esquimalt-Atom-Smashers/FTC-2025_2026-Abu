package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@Config
public class TurretSubsystem implements SubsystemBase {

    public static class Params {
        public double kLeadGainSeconds = 0;
        public double kVRobotRotation = 0;
        public double kSCoilFFComp = 0.095;
        public double kSTurretRing = 0.0;
        public double START_ANGLE_COIL_FF_COMP = 45.0;

        public double SERVO_MIN_POS = 0.0;
        public double SERVO_MAX_POS = 1.0;
        public double SERVO_START_POS = 0.5;

        // mapping
        public double SERVO_MIN_POS_DEG = 100;   // pos 0.0
        public double SERVO_MAX_POS_DEG = 260;  // pos 1.0
    }

    public static Params PARAMS = new Params();

    private final OpMode opMode;
    private final RobotContainer.Alliance alliance;
    private final Pose2d goalPos;
    private ElapsedTime loopTimer;
    private String aimbotLine;
    private boolean isTelemetryEnabled = true;

    // ================= HARDWARE =================
    private Servo turretServo;

    // ================= STATE =================
    private Pose2d robotPose;
    private PoseVelocity2d robotVelocity;
    private double lastTargetAngleStoredFC = Double.NaN;

    // ================= Subsystem State =================
    public enum TurretState {
        IDLE,
        ROBOT_FRAME_LOCK,
        GOAL_LOCK
    }

    private TurretState currentState;

    // Targets
    private double targetRobotFrameDeg = 180.0;

    public TurretSubsystem(OpMode opMode, RobotContainer.Alliance alliance,
                           TurretState turretState, Pose2d pose2d, Pose2d goalPos) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.goalPos = goalPos;

        turretServo = opMode.hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(PARAMS.SERVO_START_POS);

        this.currentState = turretState;
        this.robotPose = pose2d;

        loopTimer = new ElapsedTime();
    }

    // ================= INPUT UPDATES =================
    public void updateRobotPose(Pose2d pose, PoseVelocity2d velocity2d){
        this.robotPose = pose;
        this.robotVelocity = velocity2d;
    }

    // ================= MODE SETTERS =================
    public void lockRobotFrame(double angleDeg) {
        currentState = TurretState.ROBOT_FRAME_LOCK;
        targetRobotFrameDeg = angleDeg;
    }

    public void lockFieldPoint(){
        currentState = TurretState.GOAL_LOCK;
    }

    public void idle(){
        currentState = TurretState.IDLE;
    }

    // ================= FEEDFORWARD (LOGIC RETAINED, NOT USED) =================
    private double getFeedForwardPower(boolean isTurningRight) {
        int turningDirection = isTurningRight ? 1 : -1;
        double turnFeedforward = robotVelocity.angVel * -PARAMS.kVRobotRotation;
        double wireCoilFeedforward =
                (getTurretAngleOnBot() >= PARAMS.START_ANGLE_COIL_FF_COMP)
                        ? PARAMS.kSCoilFFComp : 0;
        return PARAMS.kSTurretRing * turningDirection + turnFeedforward + wireCoilFeedforward;
    }

    // ================= GEOMETRY =================
    private double computeFieldPointAngle(){
        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        double currentTargetAngleFC = Math.atan2(dy, dx);
        double lastLoopFieldAngle = lastTargetAngleStoredFC;
        lastTargetAngleStoredFC = currentTargetAngleFC;

        double robotFrameAngle = currentTargetAngleFC - robotPose.heading.toDouble();

        if (Double.isNaN(lastLoopFieldAngle)) {
            return Math.toDegrees(robotFrameAngle);
        }

        double leadAngle =
                ((currentTargetAngleFC - lastLoopFieldAngle) / loopTimer.seconds())
                        * PARAMS.kLeadGainSeconds;

        double predictedFieldAngle = currentTargetAngleFC + leadAngle;
        double robotFrame = predictedFieldAngle - robotPose.heading.toDouble();

        aimbotLine = "goal target heading: " + Math.toDegrees(robotFrame);
        return Math.toDegrees(robotFrame);
    }

//    private double findBestTurretAngle(double robotCentricTargetAngle) {
//        double currentTurretAngle = getTurretAngleOnBot();
//
//        double bestTurretAngle = Double.NaN;
//        double bestError = Double.POSITIVE_INFINITY;
//
//        for (int k = -1; k <= 1; k++) {
//            double candidate = robotCentricTargetAngle + k * 360.0;
//
//            if (candidate < PARAMS.MIN_ANGLE_ON_BOT || candidate > PARAMS.MAX_ANGLE_ON_BOT) {
//                continue;
//            }
//
//            double error = Math.abs(candidate - currentTurretAngle);
//            if (error < bestError) {
//                bestError = error;
//                bestTurretAngle = candidate;
//            }
//        }
//
//        if (Double.isNaN(bestTurretAngle)) {
//            bestTurretAngle = Range.clip(robotCentricTargetAngle,
//                    PARAMS.MIN_ANGLE_ON_BOT, PARAMS.MAX_ANGLE_ON_BOT);
//        }
//
//        return bestTurretAngle;
//    }

    // ================= SENSORS (VIRTUALIZED) =================
    public double getTurretAngleOnBot(){
        double pos = turretServo.getPosition();
        return servoPosToAngle(pos);
    }

    // ================= SERVO MAPPING =================
    private double angleToServoPos(double angleDeg){
        angleDeg = Range.clip(angleDeg,
                PARAMS.SERVO_MIN_POS_DEG,
                PARAMS.SERVO_MAX_POS_DEG);

        double t = (angleDeg - PARAMS.SERVO_MIN_POS_DEG) /
                (PARAMS.SERVO_MAX_POS_DEG - PARAMS.SERVO_MIN_POS_DEG);

        double pos = PARAMS.SERVO_MIN_POS + t *
                (PARAMS.SERVO_MAX_POS - PARAMS.SERVO_MIN_POS);

        return Range.clip(pos, PARAMS.SERVO_MIN_POS, PARAMS.SERVO_MAX_POS);
    }

    private double servoPosToAngle(double pos){
        double t = (pos - PARAMS.SERVO_MIN_POS) /
                (PARAMS.SERVO_MAX_POS - PARAMS.SERVO_MIN_POS);

        return PARAMS.SERVO_MIN_POS_DEG +
                t * (PARAMS.SERVO_MAX_POS_DEG - PARAMS.SERVO_MIN_POS_DEG);
    }

    // ================= TUNING =================
    public double getTargetRobotFrameDeg () {
        return targetRobotFrameDeg;
    }

    public void setHeadingCompGain(double k){
        this.PARAMS.kLeadGainSeconds = k;
    }

    // ================= LOOP =================
    @Override
    public void periodic() {
        if(currentState == TurretState.IDLE){
            return;
        }

        switch(currentState){
            case ROBOT_FRAME_LOCK:
                break;

            case GOAL_LOCK:
                targetRobotFrameDeg = computeFieldPointAngle();
                break;

            default:
                return;
        }

        double servoPos = angleToServoPos(targetRobotFrameDeg);
        turretServo.setPosition(servoPos);

        loopTimer.reset();
    }

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
            if(robotVelocity != null){
                opMode.telemetry.addData("Turret velocity", Math.toDegrees(robotVelocity.angVel));
            }
            opMode.telemetry.addData("Servo Pos", turretServo.getPosition());
        }
    }

    public void addAxonLog() {
        // no-op, retained for API compatibility
    }

    @Override
    public void resetSubsystem() {}

    @Override
    public void shutDownSubsystem() {
        idle();
    }

    @Override
    public Enum<?> getState() {
        return currentState;
    }

    public void setCurrentState(TurretState turretState) {
        currentState = turretState;
    }
}
