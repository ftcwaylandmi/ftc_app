/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ZZZ.Crater.NoDrop", group="Linear Opmode")

public class CraterNoDrop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private WaylandRobot robot = new WaylandRobot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.initHW(hardwareMap);
        robot.initRobot(true);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        robot.DriveByDistance(20, "forward", .6, opModeIsActive());
        while (robot.IsBusy2()){
            telemetry.addData("Driving", "Left");
            telemetry.addData("Moving", "%d%d", robot.GetLeftCurrent(), robot.GetLeftTarget());

            telemetry.update();
        }
        robot.DriveByLeftTime(600);
        while (robot.IsBusy2()){
            telemetry.addData("Driving", "Right");
            telemetry.addData("Moving", "%d%d", robot.GetLeftCurrent(), robot.GetLeftTarget());

            telemetry.update();
        }
     //   robot.DriveByDistance(10, "forward", .6, opModeIsActive());
        // run until the end of the match (driver presses STOP)
/*        while (opModeIsActive()) {
     //   telemetry.addData("Moving", "%d%d", robot.GetLeftCurrent(), robot.GetLeftTarget());
        //robot.EnableEncoders();

        double distance = robot.GetDistance();
        int red = robot.Red();
        int green = robot.Green();
        int blue = robot.Blue();
        telemetry.addData("distance", "%f", distance);
        telemetry.addData("red", "%d", red);
        telemetry.addData("blue", "%d", blue);
        telemetry.addData("green", "%d", green);

        telemetry.update();
        /*
        robot.DropMarker();
        robot.DriveByLeftTime(1280);
        robot.DriveByTime(robot.cm_to_ms(250));
*/
        /*
        telemetry.addData("Done Moving","Complete");
        sleep(100);
        robot.DriveByLeftTime(1300);

        while (robot.IsBusy2()) {
            telemetry.addData("Running", "Driving");
        }
        sleep(100);
        robot.DriveByTime(1800);
        while (robot.IsBusy2()) {
            telemetry.addData("Running", "Driving");
        }
        sleep(100);
        telemetry.addData("Dropping Marker", "Driving");
        robot.DropMarker();
        sleep(100);
        robot.DriveByRightTime(2100);



            /*
            if (robot.DetectBall()) {
                // run it over.
                // go to collect point
            } else {
                // move to spot b
                if (robot.DetectBall()) {
                    // run it over
                    // go to collect point
                } else {
                    // go to other ball and run it over
                    // go to collect point
                }
            }
            /*
            robot.DriveForwardWithEncoder(33);
            //turn left
            robot.DriveForwardWithEncoder(200);
            // Drop marker
            // Turn around
            robot.DriveForwardWithEncoder(500);
            // Complete

            */
       // }
    }
}
