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
    private Servo cclamp = null;
    private Clamp clamp = null;

    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;
    double armPower = 0;

    static final double SERVO_STOP = 0.5;
    static final double SERVO_ROTATE_CLOCKWISE = 1;
    static final double SERVO_ROTATE_ANTI_CLOCKWISE = 0;
    static final double powerFactor = 0.3;

    static final double MOVE_SERVO = 1;
    static final double STOP_SERVO = 0.5;
    static final double OPEN_CLAMP = 1;     // Clamp open position
    static final double CLOSE_CLAMP = 0;    // Clamp close position
    static final int    SLEEP_FOR_SERVO_MOTORS = 200;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Robot(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        clamp = new Clamp(mainHardwareMap);

        //Initialize hardware map
        topDrive  = hardwareMap.get(DcMotor.class, "topDrive");
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        clamp.Initialize();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void wait(int interval){
        sleep(interval);
    }

    public void moveArmDown(int interval) {
        armPower = -0.1;
        startArm();
        wait(interval);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
        wait(1);
        stopArm();
    }

    public void moveArmUp(int interval) {
        armPower = 0.1;
        startArm();
        wait(interval);
        stopArm();
    }

    public void holdArmUp(){
        arm.setPower(0.2);
    }

    private void stopArm(){
        armPower = 0;
        arm.setPower(armPower);
    }

    public void CloseOrOpen(boolean userInput)
    {
        clamp.CloseOrOpen(userInput);
    }

    private void startArm(){
        arm.setPower(armPower * powerFactor);
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

    public void AutoRing0() {
        moveForward(2200);

        //Drop the wobble
        moveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        moveBackward(500);
        holdArmUp();
        wait(2000);
        strafeRight(500);
        moveForward(1000);
    }

    public void AutoRing1() {
        //Move Kiwi to the desired block
        moveForward(3000);
        strafeRight(1150);

        //Drop the wobble
        moveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        holdArmUp();
        wait(2000);

        //Move Kiwi to the launch zone
        strafeLeft(1000);
    }

    public void AutoRing4() {
         //Move Kiwi to the desired block
         moveForward(4000);

         //Drop the wobble
        moveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        holdArmUp();

        //Move Kiwi to the launch zone
        moveBackward(1500);
    }
}
