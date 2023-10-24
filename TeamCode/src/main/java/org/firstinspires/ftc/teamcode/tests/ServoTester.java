package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    boolean controllerMode = true;


    public static double servoAngle = 0.0;
    public static int servoNumber = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityServo> servos = new ArrayList<>();

        ButtonToggle toggleRightBumper = new ButtonToggle();
        ButtonToggle buttonX = new ButtonToggle();
        ButtonToggle buttonY = new ButtonToggle();

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

        waitForStart();

        while (!isStopRequested()) {
            if (toggleRightBumper.isClicked(gamepad1.right_bumper)) {
                controllerMode = !controllerMode;
            }

            if (controllerMode) {
                robot.update();
                numLoops ++;

                if (gamepad1.a) {
                    servoPos[servoIndex] += 0.001;
                }
                if (gamepad1.b){
                    servoPos[servoIndex] -= 0.001;
                }
                servoPos[servoIndex] = Utils.minMaxClip(servoPos[servoIndex], 0.0, 1.0);

                // figuring out time to set servo pos
                long start = System.nanoTime();
                servos.get(servoIndex).setTargetPose(servoPos[servoIndex],1.0);
                double elapsedTime = (System.nanoTime()-start)/1000000000.0;
                totalTime += elapsedTime;

                // incrementing / decrementing servoIndex
                if (buttonX.isToggled(gamepad1.x)) {
                    servoIndex -= 1;
                }

                if (buttonY.isToggled(gamepad1.y)) {
                    servoIndex += 1;
                }

                // if the servoIndex exceeds servoSize wrap around
                servoIndex = Math.abs(servoIndex) % servoSize;

                telemetry.addData("servoNum", servoIndex);
                telemetry.addData("servoPos", servoPos[servoIndex]);
                telemetry.addData("averageServoTime", totalTime/numLoops);
            } else {
                servos.get(servoNumber).setTargetAngle(Math.toRadians(servoAngle), 1.0);
                telemetry.addData("servoAngle", servoAngle);
                telemetry.addData("servoNumber", servoNumber);
            }
            telemetry.update();
        }
    }
}
