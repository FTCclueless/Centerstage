package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoAxonEnc;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    public static double intakePower = 1.0;
    public static boolean usePosition = false;
    public static double position = Math.toRadians(60);

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityServo> servos = new ArrayList<>();

        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();
        ButtonToggle left_bumper = new ButtonToggle();

        int servoSize = 0;
        boolean intakeOn = false;

        // getting number of servos we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof PriorityServo) {
                servos.add((PriorityServo) device);
                servoSize++;
            }
        }

        double[] servoPos = new double[servoSize];
        for (int i = 0; i < servoSize; i ++){
            servoPos[i] = servos.get(i).basePos;
        }

        int servoIndex = 0;
        double numLoops = 0;
        double totalTime = 0;

        TelemetryUtil.setup();

        waitForStart();
        while (!isStopRequested()) {
            START_LOOP();
            hardwareQueue.update();
            robot.sensors.update();

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
            servos.get(servoIndex).setTargetPose(usePosition ? position : servoPos[servoIndex], 1.0);
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

            if (left_bumper.isClicked(gamepad1.left_bumper)) {
                if (intakeOn) {
                    robot.intake.intake.setTargetPower(0.0);
                } else {
                    robot.intake.intake.setTargetPower(intakePower);
                }
                intakeOn = !intakeOn;
            }

            telemetry.addData("servoName", servos.get(servoIndex).name);
            telemetry.addData("servoIndex", servoIndex);
            telemetry.addData("servoPos", servoPos[servoIndex]);
            telemetry.addData("averageServoTime", totalTime/numLoops);
            //telemetry.addData("v4Encoder", v4Bar);
            telemetry.addData("angle", servos.get(servoIndex).getCurrentAngle());
            telemetry.addData("targetAngle", servos.get(servoIndex).getTargetAngle());
            telemetry.addData("intakeColorSensorDist", robot.intake.forcePullColorSensorDist());

            if (servos.get(servoIndex) instanceof PriorityServoAxonEnc) {
                telemetry.addData("voltage", " " + ((PriorityServoAxonEnc) servos.get(servoIndex)).getEncoderVoltage());
                telemetry.addData("angle", " " + ((PriorityServoAxonEnc) servos.get(servoIndex)).getEncoderAngle());
            }

            TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
            TelemetryUtil.sendTelemetry();
            telemetry.update();
        }
    }
}
