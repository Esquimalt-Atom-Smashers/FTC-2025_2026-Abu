package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_SHOOT_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_SHOOT_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_FAR_SHOOT_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P1_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P1_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P1_Y;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P2_HEADING;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P2_X;
import static org.firstinspires.ftc.teamcode.utilities.Property.RED_THIRD_INTAKE_P2_Y;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AutoActions {
    RobotContainer robotContainer;
    DriveSubsystem drivebase;
    ShooterSubsystem shooterSubsystem;
    IntakeTransferSubsystem intakeTransferSubsystem;
    VisionSubsystem visionSubsystem;

    public AutoActions(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        drivebase = robotContainer.drivebase;
        shooterSubsystem = robotContainer.shooter;
        intakeTransferSubsystem = robotContainer.intake;
        visionSubsystem = robotContainer.vision;
    }

    public Action redFarShootAction() {
        return drivebase.getMecanumDrive().actionBuilder(drivebase.getPose()).
                strafeToLinearHeading(new Vector2d(RED_FAR_SHOOT_X, RED_FAR_SHOOT_Y), Math.toRadians(RED_FAR_SHOOT_HEADING)).build();
    }

    public Action redThirdIntakeAction() {
        return drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                .strafeToLinearHeading(new Vector2d(RED_THIRD_INTAKE_P1_X, RED_THIRD_INTAKE_P1_Y), Math.toRadians(RED_THIRD_INTAKE_P1_HEADING))
                .strafeToLinearHeading(new Vector2d(RED_THIRD_INTAKE_P2_X, RED_THIRD_INTAKE_P2_Y), Math.toRadians(RED_THIRD_INTAKE_P2_HEADING))
                .build();
    }

    public class ShootArtifactAction implements Action {
        public double targetVelocity;
        public ShooterSubsystem.FlywheelSetting flywheelSetting;
        public ElapsedTime shootTimer;
        public double seconds;
        public ShootArtifactAction(double targetRPM, double seconds) {

            shooterSubsystem.setCurrentState(ShooterSubsystem.ShooterState.MANUAL);
            flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRPM, 0);
            shooterSubsystem.shoot(flywheelSetting);

            intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
            shootTimer = new ElapsedTime();
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooterSubsystem.shoot(flywheelSetting);
            if (shootTimer.seconds() >= seconds) {
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
                return false;
            } else {
                return true;
            }
        }

        public double getSeconds() {
            return seconds;
        }
        }

    public ShootArtifactAction shootArtifactAction(double targetRPM, double seconds) {return new ShootArtifactAction(targetRPM, seconds);}
}
