package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Dual Driver2", group="OpMode")
public class DualDriverTeleOp2 extends OpMode {
    HardwarePushbotA1 robot = new HardwarePushbotA1();
    double          clawOffset  = 0.0;
    final double    CLAW_SPEED  = 0.08;                 // sets rate to move servo
    double          lastElbow = 0;
    final int       ELBOW_SPEED = 2;
    double          elbowval = 0.5;
    int             elbowiterdefault = 200;
    int             elbowiter = elbowiterdefault;
    final int       elbowitericr = 5;
    final double    elbowvalicr = 0.05;
    final double    servoup = 1;
    final double    motorpercentage = 0.95;
    int elbowloc = 0;
    int rangebase = -600;
    int rangescale = 1200;


    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello World");
        telemetry.update();
        robot.colorservo.setPosition(servoup);
        robot.colorservo2.setPosition(1);
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
        robot.colorservo2.setPosition(1);
        robot.colorservo.setPosition(servoup);
        robot.armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowloc = robot.armMotor2.getCurrentPosition();
        telemetry.addData("Say", "Servo at " + val);
        telemetry.update();

    }

    @Override
    public void stop() {
        //FIXME shut down all motor servos etc
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);

        robot.armMotor.setPower(0);
        robot.colorservo.setPosition(.35);
        robot.colorservo2.setPosition(1);
    }
    @Override
    public void loop() {
        double left;
        double right;
        double arm;
        double arm2;
        double turbo;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        if (gamepad1.right_bumper) {
            turbo = 1;
        } else {
            turbo = 0.75;
        }

        if (gamepad2.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            clawOffset -= CLAW_SPEED;
        if(gamepad2.dpad_right)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.dpad_left)
            clawOffset -= CLAW_SPEED;

        if (gamepad2.y) {
            rangebase = rangebase - 20;
        } else if (gamepad2.a) {
                rangebase = rangebase + 20;
        }

        if (gamepad2.x) {
            elbowval += elbowvalicr;
        } else if(gamepad2.a) {
            elbowval -= elbowvalicr;
        }

        robot.colorservo.setPosition(servoup);
        robot.colorservo2.setPosition(1);
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftservo.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightservo.setPosition(robot.MID_SERVO - clawOffset);
        robot.bottomleftservo.setPosition(robot.MID_SERVO - clawOffset);
        robot.bottomrightservo.setPosition(robot.MID_SERVO + clawOffset);
        robot.colorservo.setPosition(servoup);
        //FIXME for autonomous mode figure out fully extended.
        arm = gamepad2.left_stick_y;
        robot.armMotor.setPower(arm);
        arm2 = (gamepad2.right_stick_y);
        telemetry.addData("Servo2", "position " + robot.colorservo2.getPosition());
        telemetry.addData("Servo", "postition " + robot.colorservo.getPosition());
        telemetry.addData("Say", "arm power is at " + arm2);
        elbowloc += (ELBOW_SPEED * arm2);
        /*if (elbowloc > rangebase) {
            elbowloc = rangebase;
        }
        if (elbowloc < (rangebase - rangescale)) {
            elbowloc = (rangebase - rangescale);
        }*/
        telemetry.addData("elbow", "location: " + elbowloc);
        robot.armMotor2.setTargetPosition(elbowloc);
        if (true) {
            robot.armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor2.setPower(.40);
        } else {
            robot.armMotor2.setPower(0);
        }
        robot.leftDrive.setPower(left * turbo);
        robot.rightDrive.setPower(right * turbo);


        //Test commentFourMotorsDemo

    }
}