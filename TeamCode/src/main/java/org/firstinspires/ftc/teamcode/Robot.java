package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;
    double armpower = 0;
    double servopower = 0;
    private Servo clamp = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Robot(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;

    }

    public void wait(int interval){
        try {
            runtime.reset();
            runtime.wait(interval);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void moveArmDown(int interval) {
        armpower = 1;
        wait(interval);
    }
    public void moveArmUp(int interval) {
        armpower = -1;
        wait(interval);
    }
    public void moveClampin() {
        servopower = 0.25;
    }
    public void moveClampout() {
        servopower = -0.25;
    }

    public void stopRobot(){
        topPower = 0;
        leftPower = 0;
        rightPower = 0;
    }

    public void moveForward(int interval){
        topPower = 0;

        leftPower = -1;
        rightPower = 1;
        wait(interval);
        stopRobot();
    }
    public void moveBackward(int interval){
        topPower = 0;

        leftPower = 1;
        rightPower = -1;
        wait(interval);
        stopRobot();
    }
    public void rotate(int interval){
        topPower = 1;
        leftPower = 1;
        rightPower = 1;
        wait(interval);
        stopRobot();
    }



    public void strafeLeft(int interval){
        topPower = 1;
        leftPower = 0;
        rightPower = 0;
        runtime.reset();
        wait(interval);
        stopRobot();
    }

    public void strafeRight(int interval){
        topPower = -1;
        leftPower = 0;
        rightPower = 0;
        runtime.reset();
        wait(interval);
        stopRobot();
    }


    public void AutonA() {
        moveForward(3000);

        moveArmDown(500);
        moveClampout();
        moveArmUp(500);
    }

    public void AutonB() {
        moveForward(3500);
        strafeLeft(500);
        strafeRight(500);

        moveArmDown(500);
        moveClampout();
        moveArmUp(500);
    }
    public void AutonC() {
        moveForward(4000);

        moveArmDown(500);
        moveClampout();
        moveArmUp(500);
    }


}