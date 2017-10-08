package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by First Robotics on 9/27/2017.
 */
@Autonomous(name="sbh first", group="Pushbot")
@Disabled
public class sbh_first extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "spun  up");    //
        telemetry.update();
        sleep(30);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }

    }
