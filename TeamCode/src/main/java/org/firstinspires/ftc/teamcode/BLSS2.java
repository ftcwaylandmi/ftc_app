
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by MSRobotics13 on 11/5/2017.
 */
@Autonomous(name="BLSS2", group="OpMode")
public class BLSS2 extends LinearOpMode {

    HardwarePushbotA robot = new HardwarePushbotA();
    OpenGLMatrix lastLocation = null;

    double clawOffset = 0.0;
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    VuforiaLocalizer vuforia;
    ColorSensor color_sensor;

    int glyph;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        color_sensor = hardwareMap.colorSensor.get("sensorcolor");
        color_sensor.enableLed(false);
        robot.leftservo.setPosition(0);
        robot.rightservo.setPosition(0);
        robot.colorservo.setPosition(.30);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AaxVY3r/////AAAAGbyMA3eUvEA2q83CLpzVrelPW7cPUPzKsok3Iq+EKO4jDShw8YK9P/CxrfpNh7EQbsS8MfspG4ctHTT27mnwW62RW5WKw6e8a96Icl5tCWxuqy/bycKxeWra2nQoWC3AzwDhpYsuhUTjMkTss9TyVuXdW1KlIxqhTqUkIld4LA13l9xkQzgIS1eA4rlj3VPDWeIAIKepR5s7TKOmLTHDrMLSbjefzF2RAj73YAopvfi4heZNCHTyPLkVKW3AiAobToqX92ibMJTmgSIV0wvTYrxc6f6HQpC5AVRos11CYuYS95H4QEg9HqU8WkxipYMWZ8UGpu4BltPoqipA1lq04MCyxe/gvdW1gsewE63WXgz0\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();


        waitForStart();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        color_sensor.enableLed(true);
        robot.colorservo.setPosition(-0.10);
        sleep(200);
        telemetry.addData("Blue", "%s", color_sensor.blue());
        telemetry.addData("Red", "%s", color_sensor.red());
        if( color_sensor.red() > 5) {
            robot.leftDrive.setPower(0.15);
            robot.rightDrive.setPower(0.15);
            sleep(50);
            robot.leftDrive.setPower(-0.15);
            robot.rightDrive.setPower(-0.15);
            sleep(50);
        } else if (color_sensor.blue() > 5) {
            robot.leftDrive.setPower(-0.15);
            robot.rightDrive.setPower(-0.15);
            sleep(50);
            robot.leftDrive.setPower(0.15);
            robot.rightDrive.setPower(0.15);
            sleep(50);
        }
        robot.colorservo.setPosition(.30);

        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        if (vuMark.equals("center")) {
            //center
            telemetry.addData("Direction", "center");
            //straight



            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(400);
            //turn left
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(.5);
            sleep(8000);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(300);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);
        } else if (vuMark.equals("right")) {
            //right
            telemetry.addData("Direction", "right");
            //straight
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(400);
            //turn left
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(.5);
            sleep(7800);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(300);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);

        } else {
            //left
            telemetry.addData("Direction", "default left");
            //straight
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(400);
            //turn left
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(.5);
            sleep(8200);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(300);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);
        }
        //telemetry.update();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
















