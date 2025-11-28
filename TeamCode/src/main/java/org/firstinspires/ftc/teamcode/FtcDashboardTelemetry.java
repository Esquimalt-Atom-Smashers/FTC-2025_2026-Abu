package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FtcDashboardTelemetry extends BlocksOpModeCompanion {

    @ExportToBlocks (
            comment = "",
            tooltip = "show your telemetry in ftc dashboard",
            parameterLabels = {"telemetry"}
    )
    public static void enableDashTelemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}
