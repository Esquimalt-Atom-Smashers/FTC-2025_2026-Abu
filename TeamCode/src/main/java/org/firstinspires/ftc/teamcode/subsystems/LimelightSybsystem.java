package org.firstinspires.ftc.teamcode.subsystems;

import android.sax.StartElementListener;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSybsystem extends SubsystemBase {
    private Limelight3A limelight3A;
    private String LIMELIGHT_NAME = "limelight";

    private int RED_GOAL_APRILTAG_PIPELINE = 0;
    private int BLUE_GOAL_APRILTAG_PIPELINE = 1;
    private int PULL_RATE_HZ = 10;

    private Pose2d RED_GOAL_POSE = new Pose2d(-60, 60 ,0);
    private Pose2d BLUE_GOAL_POSE = new Pose2d(-60, 60, 0);
    private double METER_TO_INCH = 39.37008;

    private boolean isRedAlliance;
    private OpMode opMode;

    private Pose3D botPose;
    private double ty;

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

    public double getDistanceFromGoal() {
        Pose2d targetGoalPose = isRedAlliance? RED_GOAL_POSE : BLUE_GOAL_POSE;
        double xDistance = (botPose.getPosition().x * METER_TO_INCH) - (targetGoalPose.position.x);
        double yDistance = (botPose.getPosition().y * METER_TO_INCH) - (targetGoalPose.position.y);
        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }

    public double getTy() {
        return ty;
    }

    public void periodic() {
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            ty = result.getTy();
            botPose = result.getBotpose();
            opMode.telemetry.addLine("Limelight got data");
        } else {
            opMode.telemetry.addLine("Limelight no data");
        }

    }
}
