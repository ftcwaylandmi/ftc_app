package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by First Robotics on 11/5/2017.
 */
@Autonomous(name="right red left safe zone T 1 ", group="OpMode")
@Disabled
public class RRLS1 extends LinearOpMode {
    HardwarePushbotA robot = new HardwarePushbotA();
    double clawOffset = 0.0;
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();


        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(200); // sleeping 200ms
        robot.leftDrive.setPower(0);
        sleep(40);
        robot.leftDrive.setPower(1);
        sleep(100);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

}
