package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by MSRobotics13 on 10/1/2017.
 */

@TeleOp(name="Demo-jdd", group="OpMode")
public class jdddemo extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello World");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        telemetry.addData("Say", "Waiting for user command.");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Say", "Start initiated");
        telemetry.update();

    }

    @Override
    public void stop() {
        //FIXME shut down all motor servos etc
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        //  robot.leftArm.setPower(0);
    }
    @Override
    public void loop() {
        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.leftArm.setPower(left);

    }
    public void driveStop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void driveForward() {
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);
    }
}
