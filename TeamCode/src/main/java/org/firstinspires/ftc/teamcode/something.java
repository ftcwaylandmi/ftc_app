package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by MSRobotics13 on 10/3/2017.
 */
@TeleOp(name="random",group="a_name")
//@Diabled
public class something extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Error Virus Detected");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Say", "Error Unkown Command");

        telemetry.update();

    }

    @Override
    public void start () {
        telemetry.addData ("Say", "Starting Defense Protocol");
        telemetry.update();

    }

    public void driveforward() {
        robot.rightDrive.setPower(1);
        robot.leftDrive.setPower(-1);
    }


    @Override
    public void loop(){

    }
    @Override
    public void stop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }



}