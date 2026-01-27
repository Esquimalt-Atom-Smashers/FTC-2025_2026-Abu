package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@TeleOp(name = "Turret Tuner")
@Config
public class TurretTuner extends OpMode {

    public static class Params {
        public double targetAngleDeg = 0.0;
        public double stepDeg = 5.0;
    }
    private Params PARAMS = new Params();

    private ShooterSubsystem shooterSubsystem;
    private ElapsedTime loopTimer;

    @Override
    public void init() {
        shooterSubsystem = new ShooterSubsystem(this, RobotContainer.Alliance.RED, ShooterSubsystem.ShooterState.AUTO, new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        // Manual nudging (useful for sanity checks)
        if (gamepad1.dpad_left) {
            PARAMS.targetAngleDeg -= PARAMS.stepDeg;
        } else if (gamepad1.dpad_right) {
            PARAMS.targetAngleDeg += PARAMS.stepDeg;
        }
        shooterSubsystem.setTargetAngle(PARAMS.targetAngleDeg);
        shooterSubsystem.updateTurret();

        telemetry.addData("Target Angle (deg)", PARAMS.targetAngleDeg);
        telemetry.addData("Current Angle (deg)", shooterSubsystem.getCurrentAngle());
        telemetry.addData("Error (deg)", PARAMS.targetAngleDeg - shooterSubsystem.getCurrentAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterSubsystem.shutDownSubsystem();
    }
}
