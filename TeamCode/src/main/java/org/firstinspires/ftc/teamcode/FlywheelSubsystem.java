//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
//import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
//
//public class FlywheelSubsystem extends BlocksOpModeCompanion {
//    private static DcMotorEx flywheelMotor;
//    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
//    private static final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
//    public final double RPM_TOLERANCE = 25.0;
//
//    private static final double TICKS_PER_ROTATION = 28;
//    private static double targetVelocity; //ticks per second
//
//    private static Servo hoodAngleServo;
//    private static final String HOOD_ANGLE_SERVO_NAME = "hoodAngleServo";
//
//    private static double maxFlywheelPower = 1.0;
//
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "initializeFlywheelSubsystem",
//            parameterLabels = {}
//    )
//
//    public static void initializeFlywheelSubsystem() {
//        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
//        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
//        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        hoodAngleServo = opMode.hardwareMap.get(Servo.class, HOOD_ANGLE_SERVO_NAME);
//        hoodAngleServo.setDirection(Servo.Direction.FORWARD);
//    }
//
//    //utilities
//    public double getFlywheelPower() {
//        return flywheelMotor.getPower();
//    }
//
//    public static double rpmToTps(double rpm) {
//        return rpm * TICKS_PER_ROTATION / 60;
//    }
//
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "getFlywheelRPM",
//            parameterLabels = {}
//    )
//    public static double getFlywheelRPM() {
//        return flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
//    }
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "getTargetVelocity",
//            parameterLabels = {}
//    )
//    public static double getTargetVelocity() {return targetVelocity;}
//
//    //custom PID
//    public void setFlywheelMaxPower(double maxPower) {
//        this.maxFlywheelPower = maxPower;
//    }
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "setTargetVelocity",
//            parameterLabels = {"rpm"}
//    )
//    public static void setFlywheelTargetVelocity(double rpm) {
//        targetVelocity = rpmToTps(-rpm);
//    }
//
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "setFlywheelMotorPower",
//            parameterLabels = {"power"}
//    )
//    public static void setFlywheelMotorPower(double power) {
//        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//        flywheelMotor.setPower(power);
//    }
//
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "setHoodAngle",
//            parameterLabels = {"hoodAngle"}
//    )
//    public static void setHoodAngle(double hoodAngle) {
//        hoodAngleServo.setPosition(hoodAngle);
//    }
//
//    @ExportToBlocks(
//            comment = "",
//            tooltip = "getHoodAngle",
//            parameterLabels = {""}
//    )
//    public static double getHoodAngle() {
//        return hoodAngleServo.getPosition();
//    }
//}
