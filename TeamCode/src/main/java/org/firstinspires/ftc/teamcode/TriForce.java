package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


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
        //Gamepad data
        float gamepad1_left_stick_y = 0;
        float gamepad1_left_stick_x = 0;
        float gamepad1_right_stick_y = 0;
        float gamepad1_right_stick_x = 0;
        float gamepad1_right_trigger = 0;
        float gamepad1_left_trigger = 0;
        boolean gamepad1_left_bumper = false;
        boolean gamepad1_left_bumperPrev = false;
        boolean gamepad1_right_bumper = false;
        boolean gamepad1_right_bumperPrev = false;

        boolean gamepad1_dpad_up = false;
        boolean gamepad1_dpad_upPrev = false;
        boolean gamepad1_dpad_down = false;
        boolean gamepad1_dpad_downPrev = false;
        boolean gamepad1_dpad_left = false;
        boolean gamepad1_dpad_leftPrev = false;
        boolean gamepad1_dpad_right = false;
        boolean gamepad1_dpad_rightPrev = false;

        boolean gamepad1_a = false;
        boolean gamepad1_aPrev = false;
        boolean gamepad1_b = false;
        boolean gamepad1_bPrev = false;
        boolean gamepad1_x = false;
        boolean gamepad1_xPrev = false;
        boolean gamepad1_y = false;
        boolean gamepad1_yPrev = false;

        telemetry.addData("WAIT", "Please wait for the robot initialization...");
        telemetry.update();

        //Create RingDetection object.
        //RingDetection ringDetection = new RingDetection(hardwareMap, telemetry, runtime);
        Robot robot = new Robot(hardwareMap, telemetry, runtime, powerFactor);

        //Initialize the robot object
        robot.Init();

        telemetry.addData("READY", "Now press the PLAY button...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get the robot location.
            //robot.GetRobotPosition();

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

            gamepad1_right_trigger = gamepad1.right_trigger;
            gamepad1_left_trigger = gamepad1.left_trigger;

            gamepad1_dpad_up = gamepad1.dpad_up;
            gamepad1_dpad_down = gamepad1.dpad_down;
            gamepad1_dpad_left = gamepad1.dpad_left;
            gamepad1_dpad_right = gamepad1.dpad_right;

            if(gamepad1_left_stick_y != 0)
            {
                robot.ForwardOrBackward(gamepad1_left_stick_y);
            }
            else if(gamepad1_left_stick_x != 0)
            {
                robot.StrafeLeftOrRight(gamepad1_left_stick_x);
            }
            else if (gamepad1_right_stick_y != 0)
            {
                robot.MoveLeftAxis(gamepad1_right_stick_y);
            }
            else if(gamepad1_right_stick_x != 0)
            {
                robot.Rotate(gamepad1_right_stick_x);
            }
            else if(!gamepad1_dpad_leftPrev && gamepad1_dpad_left)
            {
                robot.ArmForceDown();
            }
            else if(!gamepad1_dpad_upPrev && gamepad1_dpad_up)
            {
                robot.RotateIntakeInReverseDirection();
            }
            else if(gamepad1_dpad_upPrev && !gamepad1_dpad_up)
            {
                robot.StopIntake();
            }
            else if(!gamepad1_dpad_downPrev && gamepad1_dpad_down)
            {
                robot.RotateIntakeInForwardDirection();
            }
            else if(gamepad1_dpad_downPrev && !gamepad1_dpad_down)
            {
                robot.StopIntake();
            }
            else if(!gamepad1_aPrev && gamepad1_a)
            {
                robot.StartOrStopIntake();
            }
            else if(!gamepad1_yPrev && gamepad1_y)
            {
                robot.StartOrStopShooter();
            }
            else if(!gamepad1_xPrev && gamepad1_x)
            {
                robot.FireRingsAtTopLevel();
            }
            else if(!gamepad1_bPrev && gamepad1_b)
            {
                robot.FireRingsAtPowerShot();
            }
            else if(gamepad1_right_trigger > 0)
            {
                robot.ArmUpOrDown();
            }
            else if(!gamepad1_right_bumperPrev && gamepad1_right_bumper)
            {
                robot.OpenOrCloseClamp();
            }
            else if(gamepad1_left_trigger > 0)
            {
                //Open for any control;
            }
            else if(!gamepad1_left_bumperPrev && gamepad1_left_bumper)
            {
                //Open for any control;
            }
            else
            {
                robot.StopRobot();
                robot.ArmStop();
            }
            gamepad1_dpad_leftPrev = gamepad1_dpad_left;
            gamepad1_dpad_upPrev = gamepad1_dpad_up;
            gamepad1_dpad_downPrev = gamepad1_dpad_down;
            gamepad1_aPrev = gamepad1_a;
            gamepad1_yPrev = gamepad1_y;
            gamepad1_xPrev = gamepad1_x;
            gamepad1_bPrev = gamepad1_b;
            gamepad1_left_bumperPrev = gamepad1_left_bumper;
            gamepad1_right_bumperPrev = gamepad1_right_bumper;

            sleep(SLEEP_TRIFORCE);
        }
    }

}
