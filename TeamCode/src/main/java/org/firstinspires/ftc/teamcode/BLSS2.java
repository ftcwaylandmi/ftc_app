
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
    //colordrivetime is the amount of ms that it will drive forward/backwards in an attempt to knock off ball0
    final int colordrivetime = 500;
    // Color sensor value - number for what the color value should be
    final double colorvalue = 5;
    // Color Arm Up
    final double colorarmup = 30;
    // Color Arm Down
    final double colorarmdown = -0.60;
    // Color drive speed
    final double colordrivespeed = 0.35;
    // turn speeed
    final double turnspeed = 0.50;
    // handopen
    final double handopen = -0.70;
    // Center Rotate Time
    final int centerrotatetime = 4800;
    // Left Rotate Time
    final int leftrotatetime = centerrotatetime + 200;
    // Right Rotate Time
    final int rightrotatetime = centerrotatetime - 200;
    // Drive Forward Time (after rotation)
    final int driveforwardtime = 300;

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
        robot.leftservo.setPosition(-.5);
        robot.rightservo.setPosition(.5);
        robot.colorservo.setPosition(colorarmup);

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
        robot.colorservo.setPosition(colorarmdown);
        sleep(200);
        telemetry.addData("Blue", "%s", color_sensor.blue());
        telemetry.addData("Red", "%s", color_sensor.red());
        if( color_sensor.red() > colorvalue) {
            //robot.leftDrive.setPower(colordrivespeed);
            robot.rightDrive.setPower(colordrivespeed);
            sleep(colordrivetime);
            //robot.leftDrive.setPower(-colordrivespeed);
            robot.rightDrive.setPower(-colordrivespeed);
            sleep(colordrivetime);
        } else if (color_sensor.blue() > colorvalue) {
            //robot.leftDrive.setPower(-colordrivespeed);
            robot.rightDrive.setPower(-colordrivespeed);
            sleep(colordrivetime);
            //robot.leftDrive.setPower(colordrivespeed);
            robot.rightDrive.setPower(colordrivespeed);
            sleep(colordrivetime);
        }
        robot.colorservo.setPosition(colorarmup);

        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        if (vuMark.equals("center")) {
            //center
            telemetry.addData("Direction", "center");
            //turn left
            robot.leftDrive.setPower(-turnspeed);
            robot.rightDrive.setPower(turnspeed);
            sleep(centerrotatetime);
        } else if (vuMark.equals("right")) {
            //right
            telemetry.addData("Direction", "right");
            //turn left
            robot.leftDrive.setPower(-turnspeed);
            robot.rightDrive.setPower(turnspeed);
            sleep(rightrotatetime);
        } else {
            //left
            telemetry.addData("Direction", "default left");
            //turn left
            robot.leftDrive.setPower(-turnspeed);
            robot.rightDrive.setPower(turnspeed);
            sleep(leftrotatetime);
        }
        //drive forward
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);
        sleep(driveforwardtime);
        //Open hands
        robot.leftservo.setPosition(handopen);
        robot.rightservo.setPosition(handopen);
        //telemetry.update();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
















