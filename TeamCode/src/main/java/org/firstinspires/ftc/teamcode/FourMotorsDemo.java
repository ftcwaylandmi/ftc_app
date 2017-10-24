package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by MSRobotics13 on 10/4/2017.
 */

public class FourMotorsDemo extends OpMode {
    HardwarePushbotA robot = new HardwarePushbotA();
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

        //  robot.leftArm.setPower(0);
    }
    @Override
    public void loop() {
        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.handServo.setPosition(robot.MID_SERVO + clawOffset);
        if (gamepad1.dpad_up) {
            robot.armMotor.setPower(.2);
        } else if (gamepad1.dpad_down) {
            robot.armMotor.setPower(-.2);
        } else {
            robot.armMotor.setPower(0);
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        //Test comment

    }
}
