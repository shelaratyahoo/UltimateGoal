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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.g
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TriForce", group="Linear Opmode")
public class TriForce extends LinearOpMode {

    static final int SLEEP_TRIFORCE = 100;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean armUpOrDown = false;
    private boolean clampOpenOrClose = false;

    // Setup a variable for each drive wheel to save power level for telemetry
    static final double powerFactor = 0.1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Create RingDetection object.
        RingDetection ringDetection = new RingDetection(hardwareMap, telemetry, runtime);
        Robot robot = new Robot(hardwareMap, telemetry, runtime, powerFactor);

        //Initialize the robot object
        robot.Init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Gamepad data
        float gamepad1_left_stick_y = 0;
        float gamepad1_left_stick_x = 0;
        float gamepad1_right_stick_y = 0;
        float gamepad1_right_stick_x = 0;
        boolean gamepad1_left_bumper = false;
        boolean gamepad1_right_bumper = false;

        boolean gamepad1_a = false;
        boolean gamepad1_b = false;
        boolean gamepad1_x = false;
        boolean gamepad1_y = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Get input from the gamepad.
            gamepad1_left_stick_y = gamepad1.left_stick_y;
            gamepad1_left_stick_x = gamepad1.left_stick_x;
            gamepad1_right_stick_y = gamepad1.right_stick_y;
            gamepad1_right_stick_x = gamepad1.right_stick_x;

            gamepad1_a = gamepad1.a;
            gamepad1_b = gamepad1.b;
            gamepad1_x = gamepad1.x;
            gamepad1_y = gamepad1.y;

            gamepad1_left_bumper = gamepad1.left_bumper;
            gamepad1_right_bumper = gamepad1.right_bumper;

            if(gamepad1_left_stick_y != 0)
            {
                robot.ForwardOrBackward(gamepad1_left_stick_y);
            }
            else if(gamepad1_left_stick_x != 0)
            {
                robot.StrafeLeftOrRight(gamepad1_left_stick_x);
            }
            else if (gamepad1_right_stick_y != 0){
                robot.MoveLeftAxis(gamepad1_right_stick_y);
            }
            else if(gamepad1_right_stick_x != 0){
                robot.Rotate(gamepad1_right_stick_x);
            }
            else if(gamepad1_left_bumper == true) {
                //robot.holdArmUp();
            }
            else if(gamepad1_right_bumper == true) {
                //robot.holdArmDown();
            }
            else if(gamepad1_a){
                //Intake on/off
//               robot.CloseOrOpen(clampOpenOrClose);
//               clampOpenOrClose = !clampOpenOrClose;
            }
            else if(gamepad1_b){

//                if(armUpOrDown){
//                    robot.holdArmUp();
//                } else {
//                    robot.downArm();
//                }
//                armUpOrDown =!armUpOrDown;
//                robot.wait(1000);
            }
            else
            {
                robot.StopRobot();
                robot.stopArm();
            }

            // Show the elapsed game time.
            telemetry.addLine("Run Time: " + runtime.toString());
            telemetry.update();
            sleep(SLEEP_TRIFORCE);
        }
    }

}
