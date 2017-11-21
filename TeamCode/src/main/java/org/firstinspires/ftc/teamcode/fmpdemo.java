package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by First Robotics on 10/1/2017.
 */

@TeleOp(name = "Demo", group = "TriggerLove1818")
@Disabled
public class fmpdemo extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();


    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Faith's example loaded");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Say", "Waiting to start");
        telemetry.update();
    }


    @Override
    public void start() {
        telemetry.addData("Say", "Start initiated");
        telemetry.update();
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);

    }

    public void driveForward() {


        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(-1);
    }

    public void driveStop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);


    }

    @Override
    public void loop() {
    }

}