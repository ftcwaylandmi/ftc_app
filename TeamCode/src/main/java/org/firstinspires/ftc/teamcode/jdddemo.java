package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbotB;

/**
 * Created by MSRobotics13 on 10/1/2017.
 */

@TeleOp(name="Demo-jdd", group="OpMode")
@Disabled
public class jdddemo extends OpMode {
    HardwarePushbotB robot = new HardwarePushbotB();
    double          clawOffset  = 0.0 ;
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo


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
        //robot.leftArm.setPower(0);
        //robot.leftClaw.setPosition(0);


    }
    @Override
    public void loop() {
        double left;
        double right;
        double in;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        //in = -gamepad1.left_stick_x;

/*
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
*/
        robot.leftDrive.setPower(left);
        //robot.rightDrive.setPower(right);
        //robot.leftArm.setPower(in);
    }
    public void driveStop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //robot.leftArm.setPower(0);
        //robot.leftClaw.setPosition(0);
    }

    public void driveForward() {
        robot.leftDrive.setPower(-1);
        robot.rightDrive.setPower(1);
        //robot.leftArm.setPower(-1);
        //robot.leftArm.setPower(1);
    }
}
