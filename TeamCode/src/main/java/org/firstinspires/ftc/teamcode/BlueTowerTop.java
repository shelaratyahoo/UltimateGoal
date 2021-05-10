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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue--TowerTop", group = "Linear Opmode")
public class BlueTowerTop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime;
    private int numberOfRings;
    static final double powerFactor = 0.7;

    public BlueTowerTop(){
        runtime  = new ElapsedTime();
        numberOfRings = 0;
    }

    @Override
    public void runOpMode() {

        log("Please wait for Tensor Flow activation...");
        //Create RingDetection object.
        RingDetection ringDetection = new RingDetection(hardwareMap, telemetry, runtime);
        Robot Autobot = new Robot(hardwareMap, telemetry, runtime, powerFactor);

        Autobot.InitAuto();

        ringDetection.activateTF();
        log("Tensor Flow activated");
        log("Now press the PLAY button...");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the loop or the driver presses STOP
        if (opModeIsActive()) {
            Autobot.autonomousMoveForward(800);
            Autobot.autonomousRotateAntiClockwise(150);

            //Detect number of rings.
            numberOfRings = ringDetection.detectRing();
            ringDetection.shutdownTF();
            log("Number of Rings=" + numberOfRings);

            //Turn ON the shooter.
            Autobot.StartOrStopShooter();

            //Move the robot to the shooting position.
            Autobot.autonomousRotateClockwise(150);
            sleep(200);
            Autobot.autonomousMoveForward(1000);
            sleep(200);
            Autobot.autonomousRotateAntiClockwise(167);
            sleep(200);
            Autobot.FireRingsAtTopLevel();
            sleep(200);

            if (numberOfRings == 0)
            {
                Autobot.AutoRing0_BlueTowerTop();
            }
            else if (numberOfRings == 1)
            {
                Autobot.AutoRing1_BlueTowerTop();
            }
            else
            {
                Autobot.AutoRing4_BlueTowerTop();
            }
        }
        ringDetection.shutdownTF();
        Autobot.ArmDown();
    }

    public void log(String logInfo)
    {
        telemetry.addLine( logInfo);
        telemetry.update();
    }

}
