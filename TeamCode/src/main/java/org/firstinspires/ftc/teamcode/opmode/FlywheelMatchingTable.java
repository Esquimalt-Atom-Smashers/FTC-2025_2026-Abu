package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
@TeleOp
public class FlywheelMatchingTable extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem(this);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a) {
                FlywheelSubsystem.FlywheelSetting flywheelSetting = flywheelSubsystem.distanceToFlywheelSetting(1.0);
                telemetry.addData("required RPM", flywheelSetting.rpm);
                telemetry.addData("required hood angle", flywheelSetting.hoodAngle);
            }
            if (gamepad1.b) {
                FlywheelSubsystem.FlywheelSetting flywheelSetting = flywheelSubsystem.distanceToFlywheelSetting(2.5);
                telemetry.addData("required RPM", flywheelSetting.rpm);
                telemetry.addData("required hood angle", flywheelSetting.hoodAngle);
            }
            if (gamepad1.x) {
                FlywheelSubsystem.FlywheelSetting flywheelSetting = flywheelSubsystem.distanceToFlywheelSetting(2.8);
                telemetry.addData("required RPM", flywheelSetting.rpm);
                telemetry.addData("required hood angle", flywheelSetting.hoodAngle);
            }

            telemetry.update();
        }
    }
}
