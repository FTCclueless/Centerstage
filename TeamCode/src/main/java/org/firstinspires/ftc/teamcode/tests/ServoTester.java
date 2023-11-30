package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    boolean controllerMode = true;

    public static double servoAngle = 0.0;
    public static int servoNumber = 0;
    public static boolean updateAll = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        HardwareQueue hardwareQueue = robot.hardwareQueue;

//        AnalogInput v4Bar = hardwareMap.get(AnalogInput.class, "V4BarServoEncoder");
//        AnalogInput botEncoder = hardwareMap.get(AnalogInput.class, "bottomTurretEncoder");
//        AnalogInput topEncoder = hardwareMap.get(AnalogInput.class, "topTurretEncoder");

        ArrayList<PriorityServo> servos = new ArrayList<>();

        ButtonToggle toggleRightBumper = new ButtonToggle();
        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        int servoSize = 0;

        // getting number of servos we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof PriorityServo) {
                servos.add((PriorityServo) device);
                servoSize++;
            }
        }

        double[] servoPos = new double[servoSize];
        for (int i = 0; i < servoSize; i ++){
            servoPos[i] = 0.5;
        }

        int servoIndex = 0;
        double numLoops = 0;
        double totalTime = 0;

        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested()) {
            robot.update();

            if (toggleRightBumper.isClicked(gamepad1.right_bumper)) {
                controllerMode = !controllerMode;
            }

            if (controllerMode) {
                numLoops ++;

                if (gamepad1.x) {
                    servoPos[servoIndex] += 0.001;
                }
                if (gamepad1.b){
                    servoPos[servoIndex] -= 0.001;
                }
                servoPos[servoIndex] = Utils.minMaxClip(servoPos[servoIndex], 0.0, 1.0);

                // figuring out time to set servo pos
                long start = System.nanoTime();
                if (updateAll) {
                    for (int i = 0; i < servoSize; i++) {
                        servos.get(i).setTargetPose(servoPos[i], 0.3);
                    }
                } else {
                    servos.get(servoIndex).setTargetPose(servoPos[servoIndex], 0.3);
                }
                double elapsedTime = (System.nanoTime()-start)/1000000000.0;
                totalTime += elapsedTime;

                // incrementing / decrementing servoIndex
                if (buttonY.isClicked(gamepad1.y)) {
                    servoIndex += 1;
                }

                if (buttonA.isClicked(gamepad1.a)) {
                    servoIndex -= 1;
                }

                // if the servoIndex exceeds servoSize wrap around
                servoIndex = Math.abs(servoIndex) % servoSize;

                telemetry.addData("servoName", servos.get(servoIndex).name);
                telemetry.addData("servoIndex", servoIndex);
                telemetry.addData("servoPos", servoPos[servoIndex]);
                telemetry.addData("averageServoTime", totalTime/numLoops);
                //telemetry.addData("v4Encoder", v4Bar);
                telemetry.addData("angle", servos.get(servoIndex).getCurrentAngle());
            } else {
                servos.get(servoNumber).setTargetAngle(Math.toRadians(servoAngle), 0.3);

                TelemetryUtil.packet.put("servoAngle", servoAngle);
                TelemetryUtil.packet.put("servoNumber", servoNumber);
                TelemetryUtil.sendTelemetry();
            }
            telemetry.update();
        }
    }
}
