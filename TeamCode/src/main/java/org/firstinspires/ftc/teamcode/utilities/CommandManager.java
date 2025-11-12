package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;

public class CommandManager {
    private DriveSubsystem driveSubsystem;
    private IntakeFeedSubsystem intakeFeedSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private LimelightSybsystem limelightSybsystem;

    public CommandManager(DriveSubsystem driveSubsystem, IntakeFeedSubsystem intakeFeedSubsystem, FlywheelSubsystem flywheelSubsystem, LimelightSybsystem limelightSybsystem) {
        this.driveSubsystem = driveSubsystem;
        this.intakeFeedSubsystem = intakeFeedSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.limelightSybsystem = limelightSybsystem;
    }

    public class ToLaunchHeading implements Action {
        private boolean cancelled = false;

        private final double ANGULAR_TOLERANCE = 1.0;//degrees

        public ToLaunchHeading() {}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if( (Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble()) <= ANGULAR_TOLERANCE) || cancelled) {
                return false;
            } else {
                double goalHeadingError = limelightSybsystem.getTy();
                if (goalHeadingError != Double.NaN) {
                    driveSubsystem.getMecanumDrive().localizer.setPose(limelightSybsystem.getBotPose2D(driveSubsystem.getCurrentPos()));

                    return driveSubsystem.getMecanumDrive().actionBuilder(driveSubsystem.getCurrentPos())
                            .turnTo(Math.toRadians(limelightSybsystem.getTy() + Math.toDegrees(driveSubsystem.getCurrentPos().heading.toDouble())))
                            .build()
                            .run(telemetryPacket);
                } else {
                    Pose2d goalPos = limelightSybsystem.getIsRedAlliance()? limelightSybsystem.RED_GOAL_POSE: limelightSybsystem.BLUE_GOAL_POSE;
                    double goalPosX = goalPos.position.x;
                    double goalPosY = goalPos.position.y;
                    double robotPosX = driveSubsystem.getCurrentPos().position.x;
                    double robotPosY = driveSubsystem.getCurrentPos().position.y;

                    double hyp = Math.sqrt(Math.pow(robotPosX - goalPosX,2) + Math.pow(robotPosY - goalPosY,2));
                    double targetHeading = Math.asin(robotPosX - goalPosX / hyp);

                    driveSubsystem.opMode.telemetry.addData("goal target heading", targetHeading);
                    return false;
//                    return mecanumDrive.actionBuilder(currentPose)
//                            .turnTo(targetHeading)
//                            .build()
//                            .run(telemetryPacket);
                }

            }
        }

        public void cancelAbruptly() {cancelled = true;}
    }

    public ToLaunchHeading toLaunchHeading() {return new ToLaunchHeading();}

    public class ShootArtifactAction implements Action {
//        TODO: adjust numbers from testing
        private final double RPM_TOLERANCE = flywheelSubsystem.RPM_TOLERANCE;
        private final double SHOOTING_RPM_DROP = -1.0;

        public double targetVelocity;
        public boolean isAccelerated;
        public ShootArtifactAction(double targetRPM, double maxPower) {

            flywheelSubsystem.setFlywheelTargetVelocity(flywheelSubsystem.rpmToTps(targetRPM));
            flywheelSubsystem.setFlywheelMaxPower(maxPower);

            intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
            intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.REVERSED);
            isAccelerated = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelSubsystem.runFlywheelControl();
            if (!isAccelerated && Math.abs(flywheelSubsystem.getFlywheelRPM() - targetVelocity) < RPM_TOLERANCE) {
                isAccelerated = true;
                intakeFeedSubsystem.setIntakePower(IntakeFeedSubsystem.ServoStates.SPINNING);
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.SPINNING);
            }
            if (isAccelerated && Math.abs(flywheelSubsystem.getFlywheelRPM() - targetVelocity) > SHOOTING_RPM_DROP) {
                return false;
            } else {
                intakeFeedSubsystem.setFeedPower(IntakeFeedSubsystem.ServoStates.REVERSED);
                return true;
            }
        }
    }

    public Action shootArtifactAction(double targetRPM, double maxPower) {return new ShootArtifactAction(targetRPM, maxPower);}
}
