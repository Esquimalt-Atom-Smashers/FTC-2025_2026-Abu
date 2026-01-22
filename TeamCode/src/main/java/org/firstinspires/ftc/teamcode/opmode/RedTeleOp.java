package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.InputUtility;
import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotPositionHolder;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

@Config
@TeleOp(name = "Kenny Red TeleOp", group = "AAA")
public class RedTeleOp extends LinearOpMode {
    RobotContainer robotContainer;
    Pose2d startingPose = new Pose2d(0, 12, Math.toRadians(90));
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    ShooterSubsystem.FlywheelSetting flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isManualRPMControl = true;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotPropertyParser.loadTeleOp();
        targetRpm = Property.CLOSE_SHOOT_RPM;
        if (RobotPositionHolder.hasData()) {
            startingPose = new Pose2d(RobotPositionHolder.getX(), RobotPositionHolder.getY(), RobotPositionHolder.getHeading());
        }
        robotContainer = new RobotContainer(this,
                startingPose,
                alliance,
                DriveSubsystem.DriveSubsystemState.TELEOP_DRIVING,
                ShooterSubsystem.ShooterState.MANUAL,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.TRACKING_GOAL);
        if (RobotPositionHolder.hasData()) {
            robotContainer.drivebase.setDriveHeadingErrorTo(Math.toRadians(90));
            RobotPositionHolder.markReceived();
        }
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

            //shooting && intake control
            if (gamepad1.right_trigger >= 0.3) {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
            } else if (gamepad1.right_bumper) {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.EJECTING);
            } else {
                robotContainer.intake.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
            }

            //flywheel control
            if (gamepad1.triangle) {
                isManualRPMControl = false;
                forceReset = true;
            } else if (gamepad1.square) {
                isManualRPMControl = true;
                forceReset = false;
            }

            if (isManualRPMControl) {
                if (gamepad1.dpad_up) {
                    targetRpm += 50;
                } else if (gamepad1.dpad_down) {
                    targetRpm -= 50;
                } else if (gamepad1.dpad_left) {
                    targetRpm = Property.CLOSE_SHOOT_RPM;
                } else if (gamepad1.dpad_right) {
                    targetRpm = Property.FAR_SHOOT_RPM;
                }
            }
            robotContainer.shoot(targetRpm, isManualRPMControl);
            if (gamepad1.left_trigger >= 0.3) {
                robotContainer.aimbotAssistedDrive(drive, strafe, turn);
            } else {
                robotContainer.drive(drive, strafe, turn);
            }
            if (gamepad1.x) {
                robotContainer.updatePoseFromVision(forceReset);
                isLLReseting = true;
            }
            robotContainer.addOpModeTelemetry("isManualRPM: " + isManualRPMControl +"\nisReseting Manual: " + isLLReseting);
            robotContainer.runRobot();
        }
        robotContainer.shutDownRobot();
        RobotPositionHolder.clear();
        Property.reset();
    }
}
