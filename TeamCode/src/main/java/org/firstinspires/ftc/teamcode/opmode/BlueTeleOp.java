package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
import org.firstinspires.ftc.teamcode.utilities.CommandManager;
import org.firstinspires.ftc.teamcode.utilities.PressAndReleaseButton;
import org.firstinspires.ftc.teamcode.utilities.RobotUtility;

@TeleOp(name = "Kenny Blue TeleOp", group = "AAA")
public class BlueTeleOp extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    IntakeFeedSubsystem intakeFeedSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    LimelightSybsystem limelightSybsystem;
    CommandManager commandManager;
    double targetRpm = PARAMS.nearRPM;

    public static class Params {
        public static double farRPM = 3500;
        public static double nearRPM = 3100;
    }
    public static Params PARAMS = new Params();
    final boolean ISREDALLIANCE = false;
    boolean isManualRPMControl = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        flywheelSubsystem = new FlywheelSubsystem(this);
        limelightSybsystem = new LimelightSybsystem(this, ISREDALLIANCE);

        Pose2d startingPose = new Pose2d(0,0,270);
        driveSubsystem = new DriveSubsystem(this, startingPose);
        driveSubsystem.setDriveHeadingError();
        commandManager = new CommandManager(driveSubsystem, intakeFeedSubsystem, flywheelSubsystem, limelightSybsystem, ISREDALLIANCE);
        waitForStart();
        flywheelSubsystem.setFlywheelTargetVelocity(-targetRpm);
        while (opModeIsActive() && !isStopRequested()) {
            //drive control
            double drive = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = RobotUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

            //reset field centric
            if (gamepad1.start || gamepad1.share) {
                driveSubsystem.setDriveHeadingError();
            }
            //reset aimbot
            if (gamepad1.back || gamepad1.share) {
                driveSubsystem.getMecanumDrive().localizer.setPose(limelightSybsystem.getBotPose2D(driveSubsystem.getCurrentPos()));
            }

            //shooting
            if (gamepad1.right_trigger >= 0.3) {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.SPINNING);
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            } else {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.REVERSED);
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            }

            //intake control
            if (gamepad1.right_bumper) {
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.REVERSED);
            }

            //flywheel control
            if (gamepad1.triangle) {
                isManualRPMControl = false;
            } else if (gamepad1.square) {
                isManualRPMControl = true;
            }
            if (isManualRPMControl) {
                if (gamepad1.dpad_up) {
                    targetRpm += 50;
                } else if (gamepad1.dpad_down) {
                    targetRpm -= 50;
                } else if (gamepad1.dpad_left) {
                    targetRpm = PARAMS.nearRPM;
                } else if (gamepad1.dpad_right) {
                    targetRpm = PARAMS.farRPM;
                }
            } else {
                FlywheelSubsystem.FlywheelSetting flywheelSetting = flywheelSubsystem.distanceToFlywheelSetting(commandManager.getDistanceToGoal());
                targetRpm = flywheelSetting.rpm;
            }
            flywheelSubsystem.setFlywheelTargetVelocity(targetRpm);

            if (gamepad1.left_trigger >= 0.3) {
                commandManager.aimbotAssistedDrive(drive, strafe);
            } else {
                driveSubsystem.drive(drive, strafe, turn);
            }
            driveSubsystem.periodic();
            flywheelSubsystem.runFlywheelControl();
            limelightSybsystem.periodic();
//            telemetry.addData("current heading", driveSubsystem.getHeading());
//            telemetry.addData("RR heading", Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()));
            telemetry.addData("distance", commandManager.getDistanceToGoal());
            telemetry.addData("targetRPM", targetRpm);
            telemetry.addData("currentRPM", flywheelSubsystem.getFlywheelRPM());
            telemetry.update();
        }
        limelightSybsystem.stop();
        RobotContainer.clear();
    }
}
