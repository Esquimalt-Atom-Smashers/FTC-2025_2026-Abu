package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp(name = "RED TeleOp", group = "AAA")
public class RedDegAdjTeleOp extends LinearOpMode {
    RobotContainer robotContainer;
    Pose2d startingPose;
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    TwoMotorShooterSubsystem.FlywheelSetting flywheelSetting = new TwoMotorShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isLeftDpadPressed = false;
    boolean isRightDpadPressed = false;
    boolean intakeToggle = false;
    boolean prevB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotPropertyParser.loadTeleOp();
        startingPose = new Pose2d(Property.TELEOP_DEFAULT_STARTING_POS_X, Property.TELEOP_DEFAULT_STARTING_POS_Y, Math.toRadians(Property.TELEOP_DEFAULT_STARTING_POS_HEADING));
        targetRpm = Property.CLOSE_SHOOT_RPM;
        if (RobotPositionHolder.hasData()) {
            startingPose = new Pose2d(RobotPositionHolder.getX(), RobotPositionHolder.getY(), RobotPositionHolder.getHeading());
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
            boolean isLLReseting = false;
            //drive control
            double drive = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = InputUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

            //reset field centric
            if (gamepad1.back || gamepad1.share) {
                robotContainer.drivebase.setHeading(alliance == RobotContainer.Alliance.RED? 90: 270);
            }
            //vision pose update
            boolean b = gamepad1.b;
            isLLReseting = b;
            if (b && !prevB) {
                robotContainer.vision.startMT2Sampling();
            }
            if (!b && prevB) {
                robotContainer.vision.stopMT2Sampling();
                robotContainer.updatePoseFromVision();
            }
            prevB = b;

            intakeToggle = gamepad1.left_bumper;

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

            if (gamepad1.y || gamepad1.triangle) {
                robotContainer.shooter.shoot(new TwoMotorShooterSubsystem.FlywheelSetting(Property.CLOSE_SHOOT_RPM, Property.CLOSE_SHOOT_HOOD_ANGLE), intakeToggle);
            } else if (gamepad1.x || gamepad1.square) {
                robotContainer.shooter.shoot(new TwoMotorShooterSubsystem.FlywheelSetting(Property.MID_SHOOT_RPM, Property.MID_SHOOT_HOOD_ANGLE), intakeToggle);
            } else if (gamepad1.a || gamepad1.cross) {
                robotContainer.shooter.shoot(new TwoMotorShooterSubsystem.FlywheelSetting(Property.FAR_SHOOT_RPM, Property.FAR_SHOOT_HOOD_ANGLE), intakeToggle);
            } else {
                robotContainer.shootByDistance();
            }
            robotContainer.drive(drive, strafe, turn);
            robotContainer.addOpModeTelemetry("is Reseting Manual: " + isLLReseting);
            robotContainer.runRobot();
        }
        robotContainer.shutDownRobot();
        RobotPositionHolder.clear();
        Property.reset();
    }
}
