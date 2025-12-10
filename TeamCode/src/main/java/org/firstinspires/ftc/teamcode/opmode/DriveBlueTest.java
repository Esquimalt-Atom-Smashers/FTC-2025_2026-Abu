package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

@TeleOp(name = "Experimental - blue", group = "test")
public class DriveBlueTest extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    IntakeFeedSubsystem intakeFeedSubsystem;
    PressAndReleaseButton buttonA;
    FlywheelSubsystem flywheelSubsystem;
    LimelightSybsystem limelightSybsystem;
    CommandManager commandManager;
    double targetRpm = 4000;

    final boolean ISREDALLIANCE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        flywheelSubsystem = new FlywheelSubsystem(this);
        limelightSybsystem = new LimelightSybsystem(this, ISREDALLIANCE);

        Pose2d startingPose;
        if (RobotContainer.hasData()) {
            startingPose = new Pose2d(RobotContainer.getX(), RobotContainer.getY(), Math.toRadians(RobotContainer.getHeading()));
            RobotContainer.markReceived();
        } else {
            startingPose = new Pose2d((72 - 7), -7, Math.toRadians(270));
        }
        driveSubsystem = new DriveSubsystem(this, startingPose);
        driveSubsystem.setDriveHeadingError();

        buttonA = new PressAndReleaseButton();
        commandManager = new CommandManager(driveSubsystem, intakeFeedSubsystem, flywheelSubsystem, limelightSybsystem, ISREDALLIANCE);
        waitForStart();
        flywheelSubsystem.setFlywheelTargetVelocity(-targetRpm);
        while (opModeIsActive() && !isStopRequested()) {
            //drive control
            double drive = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_x);
            double turn = RobotUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

            if (buttonA.getIsTrue()) {
                driveSubsystem.switchFieldCentric();
            }

            if (gamepad1.back) {
                driveSubsystem.setDriveHeadingError();
            }

            if (gamepad1.b) {
                driveSubsystem.getMecanumDrive().localizer.setPose(new Pose2d((72 - 7),-7,Math.toRadians(270)));
                driveSubsystem.setDriveHeadingErrorTo(Math.toRadians(90));
            }

            if (gamepad1.left_trigger >= 0.3) {
                driveSubsystem.changeSpeedMultiplier(1.1 - (double) gamepad1.left_trigger);
            } else {
                driveSubsystem.changeSpeedMultiplier(1.0);
            }

            //intake control
            if (gamepad1.right_bumper) {
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            } else {
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.STOPPED);
            }

            if (gamepad1.left_bumper) {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.SPINNING);
            } else {
                intakeFeedSubsystem.stopArtifact();
            }

            //flywheel control
            FlywheelSubsystem.FlywheelSetting flywheelSetting = flywheelSubsystem.distanceToFlywheelSetting(commandManager.getDistanceToGoal());
            targetRpm = flywheelSetting.rpm;
            flywheelSubsystem.setFlywheelTargetVelocity(targetRpm);

            if (gamepad1.x) {
                commandManager.aimbotAssistedDrive(drive, strafe);
            } else {
                driveSubsystem.drive(drive, strafe, turn);
            }
            driveSubsystem.periodic();
            flywheelSubsystem.runFlywheelControl();
            limelightSybsystem.periodic();
            buttonA.periodic(gamepad1.a);

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
