package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.VisionTester;
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
@Autonomous(name = "AimbotAuto", group = "AAA")
public class AimbotAuto extends LinearOpMode {
    public static class Params {
        public double turnPower = 1.0;
        public double X = 0;
        public double Y = 0;
    }
    public static Params PARAMS = new Params();
    RobotContainer robotContainer;
    Pose2d startingPose;
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    ShooterSubsystem.FlywheelSetting flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isManualRPMControl = true;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotPropertyParser.loadTeleOp();
        targetRpm = Property.CLOSE_SHOOT_RPM;
        startingPose = new Pose2d(PARAMS.X, PARAMS.Y, Math.toRadians(90));
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
            //drive control
            double drive = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = InputUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = PARAMS.turnPower;

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
            robotContainer.drivebase.aimbotAssistedDrive(drive, strafe, turn);
            if (gamepad1.x) {
                robotContainer.updatePoseFromVision(forceReset);
            }
            robotContainer.addOpModeTelemetry("isManualRPM: " + isManualRPMControl);
            robotContainer.runRobot();
        }
        robotContainer.shutDownRobot();
        RobotPositionHolder.clear();
        Property.reset();
    }
}
