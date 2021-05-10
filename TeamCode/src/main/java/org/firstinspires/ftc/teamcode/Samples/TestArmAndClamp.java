package org.firstinspires.ftc.teamcode.Samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Clamp;

@TeleOp(name="TestArmAndClamp", group = "Linear Opmode")
public class TestArmAndClamp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime;
    static final double powerFactor = 0.3;
    private Arm arm = null;
    private Clamp clamp = null;

    public TestArmAndClamp(){
        runtime  = new ElapsedTime();
    }

    @Override
    public void runOpMode() {

        //Create Arm object.
        arm = new Arm(hardwareMap, telemetry, runtime, powerFactor);
        clamp = new Clamp(hardwareMap);
        log("Arm and Clamp object created.");

        clamp.Initialize();
        arm.Initialize();
        log("Arm and Clamp Initialize...");

        //Lift the arm
        arm.UpOrDown();

        boolean gamepad1_left_bumper = false;
        boolean gamepad1_left_bumperPrev = false;
        boolean gamepad1_right_bumper = false;
        boolean gamepad1_right_bumperPrev = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the loop or the driver presses STOP
        while(opModeIsActive()) {
            gamepad1_left_bumper = gamepad1.left_bumper;
            gamepad1_right_bumper = gamepad1.right_bumper;

            if(!gamepad1_left_bumperPrev && gamepad1_left_bumper == true)
            {
                arm.UpOrDown();
            }
            else if(!gamepad1_right_bumperPrev && gamepad1_right_bumper)
            {
                clamp.OpenOrClose();
            }
            else
            {
                //arm.Stop();
                log("Arm stopped..");
            }
            gamepad1_left_bumperPrev = gamepad1_left_bumper;
            gamepad1_right_bumperPrev = gamepad1_right_bumper;
        }
    }

    public void log(String logInfo)
    {
        telemetry.addLine( logInfo);
        telemetry.update();
    }

}
