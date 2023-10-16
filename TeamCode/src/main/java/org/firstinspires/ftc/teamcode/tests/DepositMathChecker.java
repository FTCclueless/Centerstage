package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.DepositMath;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;

@TeleOp
@Config
public class DepositMathChecker extends LinearOpMode {
    public static double xError = 2;
    public static double yError = 0;
    public static double headingError = 0;
    public static double height = 10;
    public static double yOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        DepositMath depositMath = new DepositMath();
        waitForStart();

        while (!isStopRequested()) {
            depositMath.calculate(xError,yError,headingError,height,yOffset);
            Vector3 calcPos = new Vector3(0,0,0);


            double relX = depositMath.slideExtension * Math.cos(depositMath.slideAngle) + depositMath.v4BarLength * Math.cos(depositMath.v4BarYaw)*Math.cos(depositMath.v4BarPitch);
            double relY = depositMath.v4BarLength * Math.sin(depositMath.v4BarYaw)*Math.cos(depositMath.v4BarPitch);
            double relZ = depositMath.slideExtension * Math.sin(depositMath.slideAngle) + depositMath.v4BarLength * Math.sin(depositMath.v4BarPitch);

            calcPos.x = relX * Math.cos(headingError) - relY * Math.sin(headingError);
            calcPos.y = relX * Math.sin(headingError) + relY * Math.cos(headingError);
            calcPos.z = relZ;
            TelemetryUtil.packet.put("posx", calcPos.x);
            TelemetryUtil.packet.put("posy", calcPos.y);
            TelemetryUtil.packet.put("posz", calcPos.z);
            TelemetryUtil.packet.put("slide", depositMath.slideExtension);
            TelemetryUtil.packet.put("v4Yaw", depositMath.v4BarYaw);
            TelemetryUtil.packet.put("v4Pitch", depositMath.v4BarPitch);
            robot.update();
        }
    }
}
