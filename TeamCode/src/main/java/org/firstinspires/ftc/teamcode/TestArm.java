package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestArm", group = "Linear Opmode")
public class TestArm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime;
    static final double powerFactor = 0.3;
    private Arm arm = null;

    public TestArm(){
        runtime  = new ElapsedTime();
    }

    @Override
    public void runOpMode() {

        //Create Arm object.
        arm = new Arm(hardwareMap, telemetry, runtime, powerFactor);
        log("Arm object created.");

        arm.Initialize();
        log("arm Initialize.");

        //Hold the arm
//        arm.holdArmUp();
//        log("hold Arm Up");

        boolean gamepad1_left_bumper = false;
        boolean gamepad1_right_bumper = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the loop or the driver presses STOP
        while(opModeIsActive()) {
            gamepad1_left_bumper = gamepad1.left_bumper;
            gamepad1_right_bumper = gamepad1.right_bumper;
            if(gamepad1_left_bumper == true) {
                arm.holdArmUp();
                log("Arm Up");
            }
            else if(gamepad1_right_bumper == true) {
                arm.holdArmDown();
                log("Arm Down");
            }
            else
            {
                arm.stopArm();
                log("Arm stopped..");
            }
            //arm.wait(1000);
        }
    }

    public void log(String logInfo)
    {
        telemetry.addLine( logInfo);
        telemetry.update();
    }

}
