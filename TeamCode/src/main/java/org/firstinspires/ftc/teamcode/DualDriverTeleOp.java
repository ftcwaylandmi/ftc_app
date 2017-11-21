package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Dual Driver", group="OpMode")
public class DualDriverTeleOp extends OpMode {
    HardwarePushbotA robot = new HardwarePushbotA();
    double          clawOffset  = 0.0 ;
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double          lastElbow = 0;
    final double    ELBOW_SPEED = 0.10;
    double          elbowval = 0.5;
    int             elbowiter = 20;
    final int       elbowitericr = 5;
    final double    elbowvalicr = 0.05;

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
        double val = robot.leftservo.getPosition();
        telemetry.addData("Say", "Servo at " + val);
        telemetry.update();

    }

    @Override
    public void stop() {
        //FIXME shut down all motor servos etc
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);

        robot.armMotor.setPower(0);
    }
    @Override
    public void loop() {
        double left;
        double right;
        double arm;
        double arm2;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        if (gamepad2.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            clawOffset -= CLAW_SPEED;
        if(gamepad2.dpad_right)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.dpad_left)
            clawOffset -= CLAW_SPEED;

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftservo.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightservo.setPosition(robot.MID_SERVO - clawOffset);
        robot.bottomleftservo.setPosition(robot.MID_SERVO - clawOffset);
        robot.bottomrightservo.setPosition(robot.MID_SERVO + clawOffset);
        //FiXME figure out where striaght up is
        robot.colorservo.setPosition(0);
        //FIXME for autonomous mode figure out fully extended.
        arm = gamepad2.left_stick_y;
        robot.armMotor.setPower(arm);
        arm2 = gamepad2.right_stick_y;
        telemetry.addData("Say", "arm power is at " + arm2);
        if (arm2 != 0) {
            lastElbow = arm2;

        } else {
            if (lastElbow != 0 && elbowiter > 0) {
                lastElbow = elbowval;
                elbowiter--;
            } else {
                lastElbow = 0;
                elbowiter = //FIXME
            }
        }


        robot.armMotor2.setPower(lastElbow);
        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);


        //Test commentFourMotorsDemo

    }
}