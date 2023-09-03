package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryUtil {
    private static FtcDashboard dashboard;
    public static TelemetryPacket packet = new TelemetryPacket();;

    public static void setup() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public static TelemetryPacket getPacket() {
        return packet;
    }

    public static void sendTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();;
    }
}
