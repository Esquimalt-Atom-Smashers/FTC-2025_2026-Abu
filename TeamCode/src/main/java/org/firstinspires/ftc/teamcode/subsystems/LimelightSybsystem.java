package org.firstinspires.ftc.teamcode.subsystems;

import android.sax.StartElementListener;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class LimelightSybsystem extends SubsystemBase {
    private Limelight3A limelight3A;
    private String LIMELIGHT_NAME = "limelight";

    private final int RED_GOAL_APRILTAG_PIPELINE = 0;
    private final int BLUE_GOAL_APRILTAG_PIPELINE = 1;
    private final int PULL_RATE_HZ = 10;

    public final Pose2d RED_GOAL_POSE = new Pose2d(-60, 60 ,0);
    public final Pose2d BLUE_GOAL_POSE = new Pose2d(-60, -60, 0);
    private double METER_TO_INCH = 39.37008;

    private boolean isRedAlliance;
    private OpMode opMode;

    private Pose3D botPose;
    private double ty = 0.0;
    private int tagId = 0;

    public LimelightSybsystem(OpMode opMode, boolean isRedAlliance) {
        limelight3A = opMode.hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight3A.setPollRateHz(PULL_RATE_HZ);
        limelight3A.start();
        this.isRedAlliance = isRedAlliance;
        this.opMode = opMode;
        if (isRedAlliance) {
            limelight3A.pipelineSwitch(RED_GOAL_APRILTAG_PIPELINE);
        } else {
            limelight3A.pipelineSwitch(BLUE_GOAL_APRILTAG_PIPELINE);
        }
    }

    public boolean getIsRedAlliance() {return isRedAlliance;}

    public Pose2d getBotPose2D(Pose2d pose2d) {
        Pose2d returningPose = botPose != null? new Pose2d(botPose.getPosition().x * METER_TO_INCH, botPose.getPosition().y * METER_TO_INCH, botPose.getOrientation().getYaw(AngleUnit.RADIANS)): pose2d;
        if (returningPose == pose2d) {return pose2d;}
        if (isRedAlliance) {
            returningPose = new Pose2d(-returningPose.position.x, returningPose.position.y, returningPose.heading.toDouble());
        }
        return returningPose;
    }

    public double getDistanceFromGoal() {
        if (botPose != null) {
            Pose2d targetGoalPose = isRedAlliance? RED_GOAL_POSE : BLUE_GOAL_POSE;
            double xDistance = (botPose.getPosition().x * METER_TO_INCH) - (targetGoalPose.position.x);
            double yDistance = (botPose.getPosition().y * METER_TO_INCH) - (targetGoalPose.position.y);
            return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        } else {
            return 0.0;
        }
    }

    public int getTagId() {
        return tagId;
    }

    public double getTy() {
        return ty;
    }

    public void periodic() {
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            ty = result.getTy();
            botPose = result.getBotpose();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 21 || fr.getFiducialId() == 22 || fr.getFiducialId() == 33){
                    tagId = fr.getFiducialId();
                }
            }
            opMode.telemetry.addLine("Limelight got data");
        } else {
            ty = -999.0;
            botPose = null;
            tagId = 0;
            opMode.telemetry.addLine("Limelight no data");
        }
    }

    public void stop() {
        limelight3A.stop();
    }
}
