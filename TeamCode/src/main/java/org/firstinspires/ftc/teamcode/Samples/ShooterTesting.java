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

import org.firstinspires.ftc.teamcode.Shooter;


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
 * Remove or comment out the @Disabled line to add this opm ode to the Driver Station OpMode list
 */

@TeleOp(name="ShooterTesting", group="Linear Opmode")
public class ShooterTesting extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor shooter = null;
//    private Servo feeder  = null;
    static final double FEEDER_UP = 0;     // Clamp open position
    static final double FEEDER_DOWN = .5;    // Clamp close position
    private double shooterPower = 0;
    private Shooter shooter = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooter = new Shooter(hardwareMap, telemetry, runtime, 1);

        shooter.Initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean prevXButton = false;
        boolean currentXButton = false;
        boolean prevYButton = false;
        boolean currentYButton = false;
        boolean currentShooterState = false;
        float userInput  = 0;
        double F = 32767 / Shooter.SHOOTER_MAX_RPM ;
        double P = F * 0.1;
        double I = P * 0.1;
        double D = 0f;



        // run until the end of the match (driver presses STOP)
        shooter.SetVelocityPIDFCoefficients(P, I, D, F);
        shooter.SetPositionPIDFCoefficients(5.0);
        while (opModeIsActive()) {

            //Get the game pad inputs.
            currentYButton = gamepad1.y;
            currentXButton = gamepad1.x;
            userInput = gamepad1.left_stick_y;

            if(!prevYButton && currentYButton )
            {
                shooter.StartOrStop();
                currentShooterState = shooter.GetShooterState();
            }
            else if(userInput != 0)
            {
                shooter.Start(userInput);
                shooter.wait(200);
            }
            prevYButton = currentYButton;

            //Check if it rising edge of the button pressed.
            if(!prevXButton && currentXButton && currentShooterState )
            {
                //fire the rings At Top level.
                shooter.FireRingsAtTopLevel();
                //shooter.FeederTesting();
            }
            prevXButton = currentXButton;

            // Show the elapsed game time and wheel power.
            double currentVelocity = shooter.GetVelocity();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Velocity", "%.2f", currentVelocity);
            telemetry.addData("Shooter Power", "%.2f", userInput);
            telemetry.addData("Previous Button=", "%s", String.valueOf(prevXButton));
            telemetry.update();

            sleep(200);
        }

        shooter.Stop();
    }
}
