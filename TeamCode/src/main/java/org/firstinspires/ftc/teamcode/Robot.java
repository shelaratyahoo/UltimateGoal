package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import static java.lang.Thread.sleep;

import static android.os.SystemClock.sleep;

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
    static double powerFactor = 0.3;
    private Servo clamp = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Robot(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;

        //Initialize hardware map
        topDrive  = hardwareMap.get(DcMotor.class, "topDrive");
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void wait(int interval){
//        try {
//            runtime.reset();
//            runtime.wait(interval);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        sleep(interval);
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

    private void startRobot(){
        topDrive.setPower(topPower * powerFactor);
        leftDrive.setPower(leftPower  * powerFactor);
        rightDrive.setPower(rightPower * powerFactor);
    }
    public void stopRobot(){
        topPower = 0;
        leftPower = 0;
        rightPower = 0;
        topDrive.setPower(topPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void moveForward(int interval){
        topPower = 0;
        leftPower = -1;
        rightPower = 1;
        startRobot();
        wait(interval);
        stopRobot();
    }
    public void moveBackward(int interval){
        topPower = 0;
        leftPower = 1;
        rightPower = -1;
        startRobot();
        wait(interval);
        stopRobot();
    }
    public void rotate(int interval){
        topPower = 1;
        leftPower = 1;
        rightPower = 1;
        startRobot();
        wait(interval);
        stopRobot();
    }

    public void strafeLeft(int interval){
        topPower = 1;
        leftPower = 0;
        rightPower = 0;
        startRobot();
        wait(interval);
        stopRobot();
    }

    public void strafeRight(int interval){
        topPower = -1;
        leftPower = 0;
        rightPower = 0;
        startRobot();
        wait(interval);
        stopRobot();
    }


    public void AutonA() {
        moveForward(1000);
        //moveArmDown(500);
        //moveClampout();
        //moveArmUp(500);
        rotate(2000);
        //moveBackward(3000);
        //rotate
        //moveArmDown(500);
        //moveClampin();
        //moveArmUp(500);
        //rotate(2000);
        //moveForward(3000);
        //moveArmDown(500);
        //moveClampin();
        //moveArmUp(500);
    }

    public void AutonB() {
        moveForward(200);
        strafeLeft(500);
        strafeRight(500);

        moveArmDown(500);
        moveClampout();
        moveArmUp(500);

    }
    public void AutonC() {
        moveForward(3400);

        //moveArmDown(500);
        //moveClampout();
        //moveArmUp(500);
    }


}
