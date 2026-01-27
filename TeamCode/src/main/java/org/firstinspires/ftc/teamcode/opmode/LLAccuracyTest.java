package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@Autonomous(name = "RobotContainer only test", group = "AAA")
public class LLAccuracyTest extends LinearOpMode {
    RobotContainer robotContainer;
    Pose2d startingPose = new Pose2d(0, 12, Math.toRadians(90));
    RobotContainer.Alliance alliance = RobotContainer.Alliance.RED;
    double targetRpm;
    ShooterSubsystem.FlywheelSetting flywheelSetting = new ShooterSubsystem.FlywheelSetting(targetRpm, 0);
    boolean isManualRPMControl = true;
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
                robotContainer.addOpModeTelemetry(
                        "ty: " + robotContainer.vision.getTy() +
                                "\ntx: " + robotContainer.vision.getTx() +
                                "\nta: " + robotContainer.vision.getTa() +

                                "\n\nMT1" +
                                "\nx: " + poseMT1.position.x +
                                "\ny: " + poseMT1.position.y +
                                "\nheading: " + Math.toDegrees(poseMT1.heading.toDouble()) +

                                "\n\nMT2" +
                                "\nx: " + visionPos.position.x +
                                "\ny: " + visionPos.position.y +
                                "\nheading: " + Math.toDegrees(visionPos.heading.toDouble())
                );
            } else {
                robotContainer.addOpModeTelemetry("no LL data");
            }
            boolean aPressed = gamepad1.a;

            if (aPressed && !lastA && visionPos != null) {
                saveLineToFile(buildVisionLogLine(poseMT1, visionPos));
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
        return
                "ty=" + robotContainer.vision.getTy() + ", " +
                        "tx=" + robotContainer.vision.getTx() + ", " +
                        "ta=" + robotContainer.vision.getTa() + ", " +

                        "MT1[x=" + poseMT1.position.x +
                        ", y=" + poseMT1.position.y +
                        ", h=" + Math.toDegrees(poseMT1.heading.toDouble()) + "], " +

                        "MT2[x=" + visionPos.position.x +
                        ", y=" + visionPos.position.y +
                        ", h=" + Math.toDegrees(visionPos.heading.toDouble()) + "]";
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
