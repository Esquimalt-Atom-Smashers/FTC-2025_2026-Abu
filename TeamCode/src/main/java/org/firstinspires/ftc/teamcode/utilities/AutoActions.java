package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Property.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoMotorShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AutoActions {
    RobotContainer robotContainer;
    DriveSubsystem drivebase;
    TwoMotorShooterSubsystem shooterSubsystem;
    IntakeTransferSubsystem intakeTransferSubsystem;
    VisionSubsystem visionSubsystem;
    public static TrajectoryActionBuilder builder;
    public AutoActions(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        drivebase = robotContainer.drivebase;
        shooterSubsystem = robotContainer.shooter;
        intakeTransferSubsystem = robotContainer.intake;
//        visionSubsystem = robotContainer.vision;
    }
    /** IMPORTANT: Action arrangement rules:
     * Red Far Shoot -> Red Close Shoot
     * Blue Far Shoot -> Blue Close Shoot
     *
     * Red Intake Actions 1st -> 2nd -> 3rd -> Gate -> Load
     * Blue Intake Actions 1st -> 2nd -> 3rd -> Gate -> Load
     *
     * Shoot Artifact Action
     * Go to Pose Action
     * Update Pose from Vision Action
     *
     * How to add an action:
     * 1, create class:
     *      public class ___ implements Action //upper case at start
     *          public ___() //constructor
     *          public boolean run() //return false when completed
     * 2, add function:
     *      public Action ___() {return new___()} //lower case at start
    */
    //    =================================RED SHOOTING ACTIONS==========================================
    public class ShootAction implements Action {
        public Action path;
        public Action shootingAction;
        public boolean firstRun = true;
        public boolean pathNotCompleted = true;
        public boolean shootingNotCompleted = true;
        public Vector2d position;
        public double headingRad;

        public ShootAction(Vector2d position, double headingDegree) {
            this.position = position;
            this.headingRad = Math.toRadians(headingDegree);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose()).
                        strafeToLinearHeading(position, headingRad).build();
                shootingAction = shootArtifactAction(SHOOTING_SECONDS);
                firstRun = false;
            }
            pathNotCompleted = path.run(telemetryPacket);
            if (drivebase.isWithinTolerance(new Pose2d(position.x, position.y, headingRad), SHOOTING_POSITIONAL_TOLERANCE, Math.toRadians(SHOOTING_HEADING_TOLERANCE))) {
                shootingNotCompleted = shootingAction.run(telemetryPacket);
            }
            return pathNotCompleted || shootingNotCompleted;
        }
    }
    public class RedFarShootAction implements Action{
        ShootAction shootingAction;
        public RedFarShootAction() {
            shootingAction = new ShootAction(new Vector2d(RED_FAR_SHOOT_X, RED_FAR_SHOOT_Y), RED_FAR_SHOOT_HEADING);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return shootingAction.run(telemetryPacket);
        }
    }
    public Action redFarShootAction() {
        return new RedFarShootAction();
    }

    public class RedCloseShootAction implements Action{
        ShootAction shootingAction;
        public RedCloseShootAction() {
            shootingAction = new ShootAction(new Vector2d(RED_CLOSE_SHOOT_X, RED_CLOSE_SHOOT_Y), RED_CLOSE_SHOOT_HEADING);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return shootingAction.run(telemetryPacket);
        }
    }
    public Action redCloseShootAction() {
        return new RedCloseShootAction();
    }
    //    =================================BLUE SHOOTING ACTIONS==========================================
    public class BlueFarShootAction implements Action{
        ShootAction shootingAction;
        public BlueFarShootAction() {
            shootingAction = new ShootAction(new Vector2d(BLUE_FAR_SHOOT_X, BLUE_FAR_SHOOT_Y), BLUE_FAR_SHOOT_HEADING);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return shootingAction.run(telemetryPacket);
        }
    }
    public Action blueFarShootAction() {
        return new BlueFarShootAction();
    }

    public class BlueCloseShootAction implements Action{
        ShootAction shootingAction;
        public BlueCloseShootAction() {
            shootingAction = new ShootAction(new Vector2d(BLUE_CLOSE_SHOOT_X, BLUE_CLOSE_SHOOT_Y), BLUE_CLOSE_SHOOT_HEADING);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return shootingAction.run(telemetryPacket);
        }
    }
    public Action blueCloseShootAction() {
        return new BlueCloseShootAction();
    }

    //    =================================RED INTAKE ACTIONS==========================================
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

    public class RedOpenGateAction implements Action {
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(RED_GATE_INTAKE_P1_X, RED_GATE_INTAKE_P1_Y), Math.toRadians(RED_GATE_INTAKE_P1_HEADING))
                        .stopAndAdd(new SleepAction(GATE_INTAKE_DELAY_SECOND))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action redOpenGateAction() {
        return new RedOpenGateAction();
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
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action redLoadingZoneIntakeAction() {
        return new RedLoadingZoneIntakeAction();
    }



    //    =================================BLUE INTAKE ACTIONS==========================================

    public class BlueFirstIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_FIRST_INTAKE_P1_X, BLUE_FIRST_INTAKE_P1_Y), Math.toRadians(BLUE_FIRST_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(BLUE_FIRST_INTAKE_P2_X, BLUE_FIRST_INTAKE_P2_Y), Math.toRadians(BLUE_FIRST_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueFirstIntakeAction() {
        return new AutoActions.BlueFirstIntakeAction();
        }

    public class BlueSecondIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_SECOND_INTAKE_P1_X, BLUE_SECOND_INTAKE_P1_Y), Math.toRadians(BLUE_SECOND_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(BLUE_SECOND_INTAKE_P2_X, BLUE_SECOND_INTAKE_P2_Y), Math.toRadians(BLUE_SECOND_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueSecondIntakeAction() {
        return new BlueSecondIntakeAction();
    }

    public class BlueThirdIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_THIRD_INTAKE_P1_X, BLUE_THIRD_INTAKE_P1_Y), Math.toRadians(BLUE_THIRD_INTAKE_P1_HEADING))
                        .strafeToLinearHeading(new Vector2d(BLUE_THIRD_INTAKE_P2_X, BLUE_THIRD_INTAKE_P2_Y), Math.toRadians(BLUE_THIRD_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueThirdIntakeAction() {
        return new BlueThirdIntakeAction();
    }

    public class BlueLoadingZoneIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_LOAD_INTAKE_P1_X, BLUE_LOAD_INTAKE_P1_Y), Math.toRadians(BLUE_LOAD_INTAKE_P1_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueLoadingZoneIntakeAction() {
        return new BlueLoadingZoneIntakeAction();
    }

    public class BlueOpenGateAction implements Action {
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_GATE_INTAKE_P1_X, BLUE_GATE_INTAKE_P1_Y), Math.toRadians(BLUE_GATE_INTAKE_P1_HEADING))
                        .stopAndAdd(new SleepAction(GATE_INTAKE_DELAY_SECOND))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueOpenGateAction() {
        return new BlueOpenGateAction();
    }
    public class BlueGateIntakeAction implements Action{
        public Action path;
        public boolean firstRun = true;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun) {
                path = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose())
                        .strafeToLinearHeading(new Vector2d(BLUE_GATE_INTAKE_P1_X, BLUE_GATE_INTAKE_P1_Y), Math.toRadians(BLUE_GATE_INTAKE_P1_HEADING))
                        .stopAndAdd(new SleepAction(GATE_INTAKE_DELAY_SECOND))
                        .strafeToLinearHeading(new Vector2d(BLUE_GATE_INTAKE_P2_X, BLUE_GATE_INTAKE_P2_Y), Math.toRadians(BLUE_GATE_INTAKE_P2_HEADING))
                        .build();
                firstRun = false;
            }
            return path.run(telemetryPacket);
        }
    }
    public Action blueGateIntakeAction() {
        return new BlueGateIntakeAction();
    }

    //    =================================MISCELLANEOUS ACTIONS==========================================
    public class ShootArtifactAction implements Action {
        public TwoMotorShooterSubsystem.FlywheelSetting flywheelSetting;
        public ElapsedTime shootTimer;
        public double seconds;
        public boolean firstLoop = true;
        public ShootArtifactAction(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                shootTimer = new ElapsedTime();
                firstLoop = false;
                shooterSubsystem.setCurrentState(TwoMotorShooterSubsystem.ShooterState.AUTO);
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
            }
            if (shootTimer.seconds() >= seconds) {
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
                return false;
            } else {
                intakeTransferSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
                return true;
            }
        }
    }
    public ShootArtifactAction shootArtifactAction(double seconds) {return new ShootArtifactAction(seconds);}

    public Action goToPoseAction(double x, double y, double headingDegree) {
        if (builder == null) {
            builder = drivebase.getMecanumDrive().actionBuilder(drivebase.getPose());
        }
        builder = builder.strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(headingDegree));
        TrajectoryActionBuilder oldBuilder = builder;
        builder = builder.fresh(); // continue from last end
        return oldBuilder.build();
    }
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
                return failedTimes < 5;
            } else {
                return false;
            }
        }
    }
    public Action updatePoseFromVisionAction(boolean forceReset) {
        return new UpdatePoseFromVisionAction(forceReset);
    }
}
