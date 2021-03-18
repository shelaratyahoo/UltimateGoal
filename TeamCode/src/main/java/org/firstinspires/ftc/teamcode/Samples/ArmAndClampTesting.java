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

package org.firstinspires.ftc.teamcode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Clamp;

@TeleOp(name="ArmAndClampTesting", group="Linear Opmode")
public class ArmAndClampTesting extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Arm arm = null;
    private Clamp clamp = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        arm = new Arm(hardwareMap, telemetry, runtime, 1);
        clamp = new Clamp(hardwareMap);

        arm.Initialize();
        clamp.Initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean prevupbutton = false;
        boolean currentupbutton = false;
        boolean prevdownbutton = false;
        boolean currentdownbutton = false;
        boolean prevRBumper = false;
        boolean currentRBumper = false;
        boolean armPosition = false;
        final boolean ARM_UP = true;
        final boolean ARM_DOWN = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Set the motor to default speed
            currentdownbutton = gamepad1.dpad_down;
            currentupbutton = gamepad1.dpad_up;
            currentRBumper = gamepad1.right_bumper;

            if(!prevdownbutton && currentdownbutton )
            {
                arm.Down();
                armPosition = ARM_DOWN;

//                if(armPosition == ARM_DOWN){
//                    arm.Up();
//                    armPosition = ARM_UP;
//                }
//               else{
//                    arm.Down();
//                    armPosition = ARM_DOWN;
//                }
            }
            else if(!prevupbutton && currentupbutton )
            {
                arm.Up();
                armPosition = ARM_UP;

//                if(armPosition == ARM_DOWN){
//                    arm.Up();
//                    armPosition = ARM_UP;
//                }
//               else{
//                    arm.Down();
//                    armPosition = ARM_DOWN;
//                }
            }
            else {
                arm.stop();
            }
            if(!prevRBumper && currentRBumper )
            {
                clamp.CloseOrOpen(currentRBumper);
            }

//            prevdownbutton = currentdownbutton;
//            prevupbutton = currentupbutton;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Previous Button=", "%s", String.valueOf(prevupbutton));
            telemetry.update();

            sleep(200);
        }

    }


}
