
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by MSRobotics13 on 11/5/2017.
 */
@Autonomous(name="BRSS2", group="OpMode")
public class BRSS2 extends LinearOpMode {

    HardwarePushbotA robot = new HardwarePushbotA();
    OpenGLMatrix lastLocation = null;

    double clawOffset = 0.0;
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    VuforiaLocalizer vuforia;





    int glyph;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.leftservo.setPosition(0);
        robot.rightservo.setPosition(0);

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
        telemetry.addData("VuMark", "%s visible", vuMark);


        if (vuMark.equals("center")) {
            //center
            telemetry.addData("Direction", "center");
            telemetry.update();
            //straight
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(500);
            //turn left
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(-.5);
            sleep(1600);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(200);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);
        } else if (vuMark.equals("right")) {
            //right
            telemetry.addData("Direction", "right");
            telemetry.update();
            //straight
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(200);
            //turn left
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(-.5);
            sleep(1400);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(200);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);

        } else {
            //left
            telemetry.addData("Direction", "default left");
            telemetry.update();
            //straight
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(200);
            //turn left
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(-.5);
            sleep(1800);
            //drive forward
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
            sleep(200);
            //Open hands
            robot.leftservo.setPosition(.7);
            robot.rightservo.setPosition(.7);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
















