package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoMotorShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.InputUtility;
import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotPositionHolder;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

@Config
@TeleOp(name = "Kenny Red deg adjust TeleOp", group = "AAA")
public class RedDegAdjTeleOp extends LinearOpMode {
    RobotContainer robotContainer;
    Pose2d startingPose;
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    TwoMotorShooterSubsystem.FlywheelSetting flywheelSetting = new TwoMotorShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isLeftDpadPressed = false;
    boolean isRightDpadPressed = false;
    boolean intakeToggle = false;
    boolean prevLeftBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotPropertyParser.loadTeleOp();
        startingPose = new Pose2d(Property.TELEOP_DEFAULT_STARTING_POS_X, Property.TELEOP_DEFAULT_STARTING_POS_Y, Math.toRadians(Property.TELEOP_DEFAULT_STARTING_POS_HEADING));
        targetRpm = Property.CLOSE_SHOOT_RPM;
        if (RobotPositionHolder.hasData()) {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            startingPose = new Pose2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));
        }
        robotContainer = new RobotContainer(this,
                startingPose,
                alliance,
                DriveSubsystem.DriveSubsystemState.TELEOP_DRIVING,
                TwoMotorShooterSubsystem.ShooterState.DISABLED,
                TurretSubsystem.TurretState.GOAL_LOCK,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.TRACKING_GOAL);
        robotContainer.drivebase.setDriveHeadingErrorTo(Math.toRadians(90));
        waitForStart();
        while (opModeIsActive()) {
            boolean forceReset = false;
            boolean isLLReseting = false;
            //drive control
            double drive = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = InputUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

            //reset field centric
            if (gamepad1.start || gamepad1.share) {
                robotContainer.drivebase.setHeading(alliance == RobotContainer.Alliance.RED? 90: 270);
            }
            //reset aimbot
            if (gamepad1.back || gamepad1.share) {
                robotContainer.updatePoseFromVision(true);
            }

            // toggle logic
            if (gamepad1.left_bumper && !prevLeftBumper) {
                intakeToggle = !intakeToggle;
            }
            prevLeftBumper = gamepad1.left_bumper;

            // shooting && intake control
            if (gamepad1.right_trigger >= 0.3) {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
            } else if (gamepad1.right_bumper) {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.EJECTING);
            } else if (intakeToggle) {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
            } else {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.DISABLED);
            }

            //flywheel control
            if (gamepad1.triangle) {
                forceReset = true;
            } else if (gamepad1.square) {
                forceReset = false;
            }

            if (!gamepad1.dpad_left && isLeftDpadPressed) {
                robotContainer.drivebase.setHeading(Math.toDegrees(robotContainer.getPose().heading.toDouble()) + 1);
                isLeftDpadPressed = false;
            } else if (!gamepad1.dpad_right && isRightDpadPressed) {
                robotContainer.drivebase.setHeading(Math.toDegrees(robotContainer.getPose().heading.toDouble()) - 1);
                isRightDpadPressed = false;
            } else if (gamepad1.dpad_left && !isLeftDpadPressed) {
                isLeftDpadPressed = true;
            } else if (gamepad1.dpad_right && !isRightDpadPressed) {
                isRightDpadPressed = true;
            }
            robotContainer.shoot();
            robotContainer.drive(drive, strafe, turn);

            if (gamepad1.x) {
                robotContainer.updatePoseFromVision(forceReset);
                isLLReseting = true;
            }
            robotContainer.addOpModeTelemetry("is Reseting Manual: " + isLLReseting);
            robotContainer.runRobot();
        }
        robotContainer.shutDownRobot();
        RobotPositionHolder.clear();
        Property.reset();
    }
}
