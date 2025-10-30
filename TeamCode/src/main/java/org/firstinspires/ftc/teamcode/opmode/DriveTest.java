package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.utilities.PressAndReleaseButton;
import org.firstinspires.ftc.teamcode.utilities.RobotUtility;

@TeleOp(name = "Drive + Intake Test", group = "test")
public class DriveTest extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    IntakeFeedSubsystem intakeFeedSubsystem;
    PressAndReleaseButton buttonA;
    FlywheelSubsystem flywheelSubsystem;
    double targetRpm = 4000;

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this, new Pose2d(0,0,0));
        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        flywheelSubsystem = new FlywheelSubsystem(this);
        buttonA = new PressAndReleaseButton();

        waitForStart();
        flywheelSubsystem.setFlywheelTargetVelocity(4000);
        while (opModeIsActive() && !isStopRequested()) {
            double drive = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = RobotUtility.deadZoneJoyStick(gamepad1.left_stick_x);
            double turn = RobotUtility.deadZoneJoyStick(-gamepad1.right_stick_x);

            if (buttonA.getIsTrue()) {
                driveSubsystem.switchFieldCentric();
            }

            if (gamepad1.back) {
                driveSubsystem.setDriveHeadingError();
            }

            if (gamepad1.left_trigger >= 0.3) {
                driveSubsystem.changeSpeedMultiplier(1.0 - (double) gamepad1.left_trigger);
            } else {
                driveSubsystem.changeSpeedMultiplier(1.0);
            }

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

            if (gamepad1.dpad_up) {
                targetRpm += 50;
            } else if (gamepad1.dpad_down) {
                targetRpm -= 50;
            }
            flywheelSubsystem.setFlywheelTargetVelocity(targetRpm);


            while (gamepad1.x) {
                driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos())
                        .strafeToSplineHeading(new Vector2d(0, 0), 0)
                        .build();
            }

            driveSubsystem.drive(drive, strafe, turn);
            driveSubsystem.periodic();
            flywheelSubsystem.runFlywheelControl();
            buttonA.periodic(gamepad1.a);
            telemetry.update();
        }
    }
}
