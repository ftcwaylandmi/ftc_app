package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MSRobotics13 on 11/5/2017.
 */
@Autonomous(name="BLSS2", group="OpMode")
public class BLSS2 extends LinearOpMode {

    HardwarePushbotA robot = new HardwarePushbotA();
    double clawOffset = 0.0;
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void runOpMode() {
        robot.leftDrive.setPower(0.9);
        robot.rightDrive.setPower(1);
        sleep(200);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }
}

















