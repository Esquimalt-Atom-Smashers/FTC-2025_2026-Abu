package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Property.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
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

    public class RedFarShootAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose()).
                    strafeToLinearHeading(new Vector2d(RED_FAR_SHOOT_X, RED_FAR_SHOOT_Y), Math.toRadians(RED_FAR_SHOOT_HEADING)).build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redFarShootAction() {
        return new RedFarShootAction();
    }

    public Action redCloseShootAction() {
        return new RedCloseShootAction();
    }

    public class RedCloseShootAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose()).
                        strafeToLinearHeading(new Vector2d(RED_CLOSE_SHOOT_X, RED_CLOSE_SHOOT_Y), Math.toRadians(RED_CLOSE_SHOOT_HEADING)).build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public class RedFirstIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_FIRST_INTAKE_P1_X, RED_FIRST_INTAKE_P1_Y), Math.toRadians(RED_FIRST_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(RED_FIRST_INTAKE_P2_X, RED_FIRST_INTAKE_P2_Y), Math.toRadians(RED_FIRST_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redFirstIntakeAction() {
        return new RedFirstIntakeAction();
    }

    public class RedSecondIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_SECOND_INTAKE_P1_X, RED_SECOND_INTAKE_P1_Y), Math.toRadians(RED_SECOND_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(RED_SECOND_INTAKE_P2_X, RED_SECOND_INTAKE_P2_Y), Math.toRadians(RED_SECOND_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redSecondIntakeAction() {
        return new RedSecondIntakeAction();
    }

    public class RedThirdIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_THIRD_INTAKE_P1_X, RED_THIRD_INTAKE_P1_Y), Math.toRadians(RED_THIRD_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(RED_THIRD_INTAKE_P2_X, RED_THIRD_INTAKE_P2_Y), Math.toRadians(RED_THIRD_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redThirdIntakeAction() {
        return new RedThirdIntakeAction();
    }

    public class RedGateIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_GATE_INTAKE_P1_X, RED_GATE_INTAKE_P1_Y), Math.toRadians(RED_GATE_INTAKE_P1_HEADING))
                        .stopAndAdd(new SleepAction(GATE_INTAKE_DELAY_SECOND))
                        .strafeToLinearHeading(new Vector2d(RED_GATE_INTAKE_P2_X, RED_GATE_INTAKE_P2_Y), Math.toRadians(RED_GATE_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redGateIntakeAction() {
        return new RedGateIntakeAction();
    }

    public class RedLoadingZoneIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_LOAD_INTAKE_P1_X, RED_LOAD_INTAKE_P1_Y), Math.toRadians(RED_LOAD_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(RED_LOAD_INTAKE_P2_X, RED_LOAD_INTAKE_P2_Y), Math.toRadians(RED_LOAD_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action goToPoseAction(double x, double y, double headingDegree) {
        return new GoToPoseAction(x, y, headingDegree);
    }

    public class GoToPoseAction implements Action{
        public Action path;
        public boolean firstRun = true;
        public double x;
        public double y;
        public double headingDegree;

        public GoToPoseAction(double x, double y, double headingDegree) {
            this.x = x;
            this.y = y;
            this.headingDegree = headingDegree;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(headingDegree))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }

    public Action redLoadingZoneIntakeAction() {
        return new RedLoadingZoneIntakeAction();
    }

    public class ShootArtifactAction implements Action {
        public double targetVelocity;
        public ShooterSubsystem.FlywheelSetting flywheelSetting;
        public ElapsedTime shootTimer;
        public double seconds;
        public boolean firstLoop = true;
        public ShootArtifactAction(double targetRPM, double seconds) {
            flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRPM, 0);
            shooterSubsystem.setTargetSetting(flywheelSetting);

            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                shootTimer = new ElapsedTime();
                firstLoop = false;
                shooterSubsystem.setCurrentState(ShooterSubsystem.ShooterState.MANUAL);
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
            }
            shooterSubsystem.shoot(flywheelSetting);
            if (shootTimer.seconds() >= seconds) {
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
                return false;
            } else {
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
                return true;
            }
        }
    }

    public ShootArtifactAction shootArtifactAction(double targetRPM, double seconds) {return new ShootArtifactAction(targetRPM, seconds);}

    public class UpdatePoseFromVisionAction implements Action {
        public boolean forceReset;
        public int failedTimes;
        public UpdatePoseFromVisionAction(boolean forceReset) {
            visionSubsystem.setCurrentState(VisionSubsystem.VisionState.TRACKING_GOAL);
            this.forceReset = forceReset;
            this.failedTimes = 0;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean updateSuccessful = robotContainer.updatePoseFromVision(forceReset);
            if (!updateSuccessful) {
                failedTimes ++;
            }
            return updateSuccessful && failedTimes < 5;
        }
    }

    public Action updatePoseFromVisionAction(boolean forceReset) {return new UpdatePoseFromVisionAction(forceReset);}
}
