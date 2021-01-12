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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="RedTreecone", group = "Autonomous")
public class RedTreeconeAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime;
    private int numberOfRings;

    public RedTreeconeAuto(){
        runtime  = new ElapsedTime();
        numberOfRings = 0;
    }

    @Override
    public void runOpMode() {

        //Create RingDetection object.
        RingDetection ringDetection = new RingDetection(hardwareMap, telemetry, runtime);
        Robot Autobot = new Robot(hardwareMap, telemetry, runtime);
        ringDetection.activateTF();
        telemetry.addLine( "Tensor Flow activated...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //Move Kiwi to the desired block
            //Drop the wobble
            //Move Kiwi to the next wobble
            //Pickup the next wobble
            //Move Kiwi to the desired block
            //Drop the wobble
            //Move Kiwi to the launch zone
            //Detect the number of rings

            numberOfRings = ringDetection.detectRing();
            ringDetection.shutdownTF();
            telemetry.addData("Status", "Number of Rings=" + numberOfRings);
            telemetry.update();
            if (numberOfRings == 0){
                Autobot.AutonA();
            }
            else if (numberOfRings == 1){
                Autobot.AutonA();
            }
            else{
                Autobot.AutonC();
            }
        }
        ringDetection.shutdownTF();
    }
}