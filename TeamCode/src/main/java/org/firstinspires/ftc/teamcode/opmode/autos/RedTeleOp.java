package org.firstinspires.ftc.teamcode.opmode.autos;

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
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@Config
@TeleOp(name = "Kenny Red TeleOp", group = "AAA")
public class RedTeleOp extends LinearOpMode {
    RobotContainer robotContainer;
    public static class Params {
        public static double farRPM = 3500;
        public static double nearRPM = 3100;
    }
    public static Params PARAMS = new Params();

    Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(90));
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm = PARAMS.nearRPM;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotContainer = new RobotContainer(this,
                startingPose,
                alliance,
                DriveSubsystem.DriveSubsystemState.TELEOP_DRIVING,
                ShooterSubsystem.ShooterState.DISABLED,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.DISABLED);

        waitForStart();
        while (opModeIsActive()) {
            robotContainer.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robotContainer.runRobot();
        }
        robotContainer.shutDownRobot();
    }
}
