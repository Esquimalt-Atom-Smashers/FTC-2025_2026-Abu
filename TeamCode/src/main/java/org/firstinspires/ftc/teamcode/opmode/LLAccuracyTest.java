package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utilities.InputUtility;
import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;
import org.firstinspires.ftc.teamcode.utilities.RobotPositionHolder;
import org.firstinspires.ftc.teamcode.utilities.RobotPropertyParser;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Config
@TeleOp(name = "LLAccuracyTest", group = "AAA")
public class LLAccuracyTest extends LinearOpMode {
    RobotContainer robotContainer;
    Pose2d startingPose = new Pose2d(0, 12, Math.toRadians(90));
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    ShooterSubsystem.FlywheelSetting flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isManualRPMControl = true;
    ElapsedTime aHoldTimer = new ElapsedTime();
    boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotPropertyParser.loadTeleOp();
        targetRpm = Property.CLOSE_SHOOT_RPM;
        if (RobotPositionHolder.hasData()) {
            startingPose = new Pose2d(RobotPositionHolder.getX(), RobotPositionHolder.getY(), RobotPositionHolder.getHeading());
        }
        robotContainer = new RobotContainer(this,
                startingPose,
                alliance,
                DriveSubsystem.DriveSubsystemState.TELEOP_DRIVING,
                ShooterSubsystem.ShooterState.DISABLED,
                IntakeTransferSubsystem.IntakeTransferState.DISABLED,
                VisionSubsystem.VisionState.TRACKING_GOAL);
        if (RobotPositionHolder.hasData()) {
            robotContainer.drivebase.setDriveHeadingErrorTo(Math.toRadians(90));
            RobotPositionHolder.markReceived();
        }
        waitForStart();
        while (opModeIsActive()) {
            Pose2d poseMT1 = robotContainer.vision.getLimelightPosMT1();
            Pose2d visionPos = robotContainer.vision.getLimelightPosMT2();
            if (visionPos != null) {
                String timestamp = new java.text.SimpleDateFormat("HH:mm:ss.SSS").format(new java.util.Date());

                robotContainer.addOpModeTelemetry(
                        timestamp + ", " +
                                robotContainer.vision.getTy() + ", " +
                                robotContainer.vision.getTx() + ", " +
                                robotContainer.vision.getTa() + ", " +
                                robotContainer.vision.getBotPoseAvgDist() + ", " +
                                poseMT1.position.x + ", " +
                                poseMT1.position.y + ", " +
                                Math.toDegrees(poseMT1.heading.toDouble()) + ", " +
                                visionPos.position.x + ", " +
                                visionPos.position.y + ", " +
                                Math.toDegrees(visionPos.heading.toDouble())
                );
            } else {
                robotContainer.addOpModeTelemetry("no LL data");
            }
            boolean aPressed = gamepad1.a;
            if (aPressed && visionPos != null) {
                // First press (edge)
                if (!lastA) {
                    saveLineToFile(buildVisionLogLine(poseMT1, visionPos));
                    aHoldTimer.reset();
                }
                // Held down: repeat every 0.5 seconds
                else if (aHoldTimer.seconds() >= 0.5) {
                    saveLineToFile(buildVisionLogLine(poseMT1, visionPos));
                    aHoldTimer.reset();
                }
            }
            lastA = aPressed;


            robotContainer.runRobot();

            telemetry.update();
        }
        robotContainer.shutDownRobot();
        RobotPositionHolder.clear();
        Property.reset();
    }

    private String buildVisionLogLine(Pose2d poseMT1, Pose2d visionPos) {
        String timestamp = new java.text.SimpleDateFormat("HH:mm:ss.SSS").format(new java.util.Date());

        return
                timestamp + ", " +
                        robotContainer.vision.getTy() + ", " +
                        robotContainer.vision.getTx() + ", " +
                        robotContainer.vision.getTa() + ", " +
                        robotContainer.vision.getBotPoseAvgDist() + ", " +
                        poseMT1.position.x + ", " +
                        poseMT1.position.y + ", " +
                        Math.toDegrees(poseMT1.heading.toDouble()) + ", " +
                        visionPos.position.x + ", " +
                        visionPos.position.y + ", " +
                        Math.toDegrees(visionPos.heading.toDouble());
    }
    private void saveLineToFile(String line) {
        try {
            File file = new File(
                    "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/vision_log.txt"
            );

            // Ensure parent directories exist
            file.getParentFile().mkdirs();

            FileWriter writer = new FileWriter(file, true); // append mode
            writer.write(line + "\n");
            writer.close();

        } catch (IOException e) {
            telemetry.addLine("Failed to write vision_log.txt");
        }
    }
}
