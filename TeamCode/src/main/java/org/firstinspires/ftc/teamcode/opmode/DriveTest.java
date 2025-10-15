package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.utilities.PressAndReleaseButton;
import org.firstinspires.ftc.teamcode.utilities.RobotUtility;

@TeleOp(name = "Drive + Intake Test", group = "test")
public class DriveTest extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    IntakeFeedSubsystem intakeFeedSubsystem;
    PressAndReleaseButton buttonA;

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this, new Pose2d(0,0,0));
        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        buttonA = new PressAndReleaseButton();

        while (opModeIsActive() && !isStopRequested()) {
            double drive = RobotUtility.deadZoneJoyStick(-gamepad1.left_stick_y);
            double strafe = RobotUtility.deadZoneJoyStick(gamepad1.left_stick_x);
            double turn = RobotUtility.deadZoneJoyStick(gamepad1.right_stick_x);

            driveSubsystem.drive(drive, strafe, turn);
            driveSubsystem.periodic();
            buttonA.periodic(gamepad1.a);
            telemetry.update();

            if (buttonA.getIsTrue()) {
                driveSubsystem.switchFieldCentric();
            }

            if (gamepad1.right_bumper) {
                driveSubsystem.changeSpeedMultiplier(0.5);
            } else {
                driveSubsystem.changeSpeedMultiplier(1.0);
            }

            if (gamepad1.x) {
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            } else {
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.STOPPED);
            }

            if (gamepad1.y) {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.SPINNING);
            } else {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.STOPPED);
            }

            while (gamepad1.x) {
                driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos())
                        .strafeToSplineHeading(new Vector2d(0, 0), 0)
                        .build();
            }
        }
    }
}
