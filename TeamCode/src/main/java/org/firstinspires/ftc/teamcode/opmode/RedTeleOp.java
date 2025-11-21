package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
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
@Config
@TeleOp(name = "Kenny - RedTeleOp", group = "AAA")
public class RedTeleOp extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    IntakeFeedSubsystem intakeFeedSubsystem;
    PressAndReleaseButton buttonA;
    FlywheelSubsystem flywheelSubsystem;
    LimelightSybsystem limelightSybsystem;
    CommandManager commandManager;
    double targetRpm = PARAMS.farRPM;

    public static class Params {
        public static double farRPM = 3600;
        public static double nearRPM = 3100;
    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        flywheelSubsystem = new FlywheelSubsystem(this);
        limelightSybsystem = new LimelightSybsystem(this, true);

        Pose2d startingPose;
        if (RobotContainer.hasData()) {
            startingPose = new Pose2d(RobotContainer.getX(), RobotContainer.getY(), Math.toRadians(RobotContainer.getHeading()));
            RobotContainer.markReceived();
        } else {
            startingPose = limelightSybsystem.getIsRedAlliance()? RobotUtility.RED_RESET_POS: RobotUtility.BLUE_RESET_POS;
        }
        driveSubsystem = new DriveSubsystem(this, startingPose);

        buttonA = new PressAndReleaseButton();
        commandManager = new CommandManager(driveSubsystem, intakeFeedSubsystem, flywheelSubsystem, limelightSybsystem);
        waitForStart();
        flywheelSubsystem.setFlywheelTargetVelocity(-targetRpm);
        while (opModeIsActive() && !isStopRequested()) {

            //drive control
            double drive = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = RobotUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

//            if (buttonA.getIsTrue()) {
//                driveSubsystem.switchFieldCentric();
//            }

            //reset field centric
            if (gamepad1.start || gamepad1.options) {
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
            if (gamepad1.dpad_up) {
                targetRpm += 50;
            } else if (gamepad1.dpad_down) {
                targetRpm -= 50;
            } else if (gamepad1.dpad_left) {
                targetRpm = PARAMS.nearRPM;
            } else if (gamepad1.dpad_right) {
                targetRpm = PARAMS.farRPM;
            }
            flywheelSubsystem.setFlywheelTargetVelocity(targetRpm);

            if (gamepad1.left_trigger >= 0.3) {
                commandManager.aimbotAssistedDrive(drive, strafe);
            } else {
                driveSubsystem.drive(drive, strafe, turn);
            }

            if (gamepad1.b) {
                driveSubsystem.getMecanumDrive().localizer.setPose(limelightSybsystem.getIsRedAlliance()? RobotUtility.RED_RESET_POS : RobotUtility.BLUE_RESET_POS);
            }

            driveSubsystem.periodic();
            flywheelSubsystem.runFlywheelControl();
            limelightSybsystem.periodic();
            buttonA.periodic(gamepad1.a);

//            telemetry.addData("current heading", driveSubsystem.getHeading());
//            telemetry.addData("RR heading", Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()));
            telemetry.addData("targetRPM", targetRpm);
            telemetry.addData("currentRPM", flywheelSubsystem.getFlywheelRPM());
            telemetry.update();
        }
        limelightSybsystem.stop();
        RobotContainer.clear();
    }
}
