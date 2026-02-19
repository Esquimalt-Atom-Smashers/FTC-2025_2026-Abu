package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.RedDegAdjTeleOp;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@Config
public class TurretSubsystem implements SubsystemBase {

    public static class Params {
        public double kLeadGainSeconds = 1;
        public double kVRobotRotation = -0.1;
        public double kVFieldAngleChange = -1.0;

        // mapping
        public double RED_SERVO_MIN_POS_DEG = -160;   // pos 0.0
        public double RED_SERVO_MAX_POS_DEG = 0;  // pos 1.0

        public double BLUE_SERVO_MIN_POS_DEG = 20;
        public double BLUE_SERVO_MAX_POS_DEG = 180;
    }

    public static Params PARAMS = new Params();

    private final OpMode opMode;
    private final RobotContainer.Alliance alliance;
    private final Pose2d goalPos;
    public ElapsedTime loopTimer;
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
    private double servoMinPosDeg;
    private double servoMaxPosDeg;

    public TurretSubsystem(OpMode opMode, RobotContainer.Alliance alliance,
                           TurretState turretState, Pose2d pose2d, Pose2d goalPos) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.goalPos = goalPos;

        turretServo = opMode.hardwareMap.get(Servo.class, "turretServo");

        this.currentState = turretState;
        this.robotPose = pose2d;

        loopTimer = new ElapsedTime();
        if (alliance == RobotContainer.Alliance.BLUE)  {
            servoMinPosDeg = PARAMS.BLUE_SERVO_MIN_POS_DEG;
            servoMaxPosDeg = PARAMS.BLUE_SERVO_MAX_POS_DEG;
        } else {
           servoMinPosDeg = PARAMS.RED_SERVO_MIN_POS_DEG;
           servoMaxPosDeg = PARAMS.RED_SERVO_MAX_POS_DEG;
        }
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

    // ================= GEOMETRY =================
    private double computeFieldPointAngle(){
        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        double currentTargetAngleFC = Math.atan2(dy, dx);

        double leadAngle = getPredictedTransRotation();

        double predictedFieldAngle = currentTargetAngleFC + leadAngle;
        double robotFrame = predictedFieldAngle - robotPose.heading.toDouble();

        double robotFrameDeg = Math.toDegrees(robotFrame);
        robotFrameDeg = AngleUnit.normalizeDegrees(robotFrameDeg);

        robotFrameDeg = Range.clip(robotFrameDeg, Math.min(servoMinPosDeg, servoMaxPosDeg), Math.max(servoMinPosDeg, servoMaxPosDeg));

        return robotFrameDeg;
    }

    private double getPredictedTransRotation(){

        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        double fieldAngleNow = Math.atan2(dy, dx);

        if (Double.isNaN(lastTargetAngleStoredFC)) {
            lastTargetAngleStoredFC = fieldAngleNow;
            return 0.0;
        }

        double dt = loopTimer.seconds();
        if (dt < 1e-4) dt = 1e-4;
        aimbotLine = "fieldAngle now: " + Math.toDegrees(fieldAngleNow) + "\nlast field angle: " + Math.toDegrees(lastTargetAngleStoredFC);
        double dTheta = fieldAngleNow - lastTargetAngleStoredFC; // wrap-safe
        double fieldAngularVel = dTheta / dt; // rad/s

        double predictedFieldAngle = fieldAngularVel * PARAMS.kLeadGainSeconds;

        lastTargetAngleStoredFC = fieldAngleNow;
        // empirical projectile flight time
        double predictedRobotRotation = robotVelocity.angVel * PARAMS.kLeadGainSeconds * PARAMS.kVRobotRotation;

        // predicted angular offset in FIELD frame
        return predictedFieldAngle + predictedRobotRotation;
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
    private double angleToServoPos(double angleDeg) {

        double angle1 = servoMinPosDeg;
        double angle2 = servoMaxPosDeg;

        double pos1 = 0.0;
        double pos2 = 1.0;

        double denom = (angle2 - angle1);

        double pos = pos1 + ((angleDeg - angle1) * ((pos2 - pos1) / denom));

        return Range.clip(pos,
                0.0,
                1.0);
    }

    private double servoPosToAngle(double pos) {

        double angle1 = servoMinPosDeg;
        double angle2 = servoMaxPosDeg;

        double pos1 = 0.0;
        double pos2 = 1.0;

        double denom = (pos2 - pos1);

        double angle = angle1 + (pos - pos1) * (angle2 - angle1) / denom;

        return Range.clip(angle,
                Math.min(angle1, angle2),
                Math.max(angle1, angle2));
    }


    // ================= TUNING =================
    public double getTargetRobotFrameDeg () {
        return targetRobotFrameDeg;
    }

    public void setHeadingCompGain(double k){
        this.PARAMS.kLeadGainSeconds = k;
    }

    // ================= LOOP =================
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
//            opMode.telemetry.addData("Current TA On Bot", getTurretAngleOnBot());
//            opMode.telemetry.addData("Target TA On Bot", targetRobotFrameDeg);
//            opMode.telemetry.addLine(aimbotLine);
//            if(robotVelocity != null){
//                opMode.telemetry.addData("Turret velocity", Math.toDegrees(robotVelocity.angVel));
//            }
//            opMode.telemetry.addData("Servo Pos", turretServo.getPosition());
            opMode.telemetry.addData("lt", loopTimer.milliseconds());
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
