package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.HardwarePushbotA;
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="FourDrive", group="Pushbot")
public class FourMotorsDemo extends OpMode {
    HardwarePushbotA robot = new HardwarePushbotA();

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
        robot.rightDrive2.setPower(0);
        robot.leftDrive2.setPower(0);

        //  robot.leftArm.setPower(0);
    }
    @Override
    public void loop() {
        double left;
        double right;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.leftDrive2.setPower(left);
        robot.rightDrive2.setPower(right);
    }
}
