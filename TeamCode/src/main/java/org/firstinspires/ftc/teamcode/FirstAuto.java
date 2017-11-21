package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MSRobotics13 on 11/5/2017.
 */
@Autonomous(name="FirstAuto", group="OpMode")
@Disabled
public class FirstAuto extends LinearOpMode {
    HardwarePushbotA robot = new HardwarePushbotA();
    double clawOffset = 0.0;
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftservo.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightservo.setPosition(robot.MID_SERVO - clawOffset);
        robot.bottomleftservo.setPosition(robot.MID_SERVO + clawOffset);
        robot.bottomrightservo.setPosition(robot.MID_SERVO - clawOffset);


        if (gamepad1.dpad_up) {
            robot.armMotor.setPower(.2);
        } else if (gamepad1.dpad_down) {
            robot.armMotor.setPower(-.2);
        } else {
            robot.armMotor.setPower(0);
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        //Test commentFourMotorsDemo

    }
}
