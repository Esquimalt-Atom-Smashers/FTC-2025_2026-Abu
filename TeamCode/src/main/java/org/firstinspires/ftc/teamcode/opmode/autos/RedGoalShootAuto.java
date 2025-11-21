package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSybsystem;
import org.firstinspires.ftc.teamcode.utilities.CommandManager;
@Disabled
@Autonomous(name = "Experimental: RedGoalShootAuto")
public class RedGoalShootAuto extends LinearOpMode {

    public static class Params {
        public double strafe1X = 1.0;
        public double strafe1Y = 1.0;

        public double targetRPM = 4000;
    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startingPose = new Pose2d(60, 60, Math.toRadians(0));
        DriveSubsystem driveSubsystem = new DriveSubsystem(this, startingPose);
        IntakeFeedSubsystem intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem(this);
        LimelightSybsystem limelightSybsystem = new LimelightSybsystem(this, true);

        CommandManager commandManager = new CommandManager(driveSubsystem, intakeFeedSubsystem, flywheelSubsystem, limelightSybsystem);
        waitForStart();

        Actions.runBlocking(
                driveSubsystem.getMecanumDrive().actionBuilder(startingPose)
                        .strafeToConstantHeading(new Vector2d(PARAMS.strafe1X, PARAMS.strafe1Y))
                        .stopAndAdd(commandManager.shootArtifactAction(PARAMS.targetRPM, 1.0))
                        .stopAndAdd(commandManager.shootArtifactAction(PARAMS.targetRPM, 1.0))
                        .stopAndAdd(commandManager.shootArtifactAction(PARAMS.targetRPM, 1.0))
                        .build());
    }
}
