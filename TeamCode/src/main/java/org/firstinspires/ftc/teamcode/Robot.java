package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Clamp clamp = null;
    private Arm arm = null;
    private IMUSensor imuSensor = null;

    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;

    static final double SERVO_STOP = 0.5;
    static final double SERVO_ROTATE_CLOCKWISE = 1;
    static final double SERVO_ROTATE_ANTI_CLOCKWISE = 0;

    static final double MOVE_SERVO = 1;
    static final double STOP_SERVO = 0.5;
    static final double OPEN_CLAMP = 1;     // Clamp open position
    static final double CLOSE_CLAMP = 0;    // Clamp close position
    static final int    SLEEP_FOR_SERVO_MOTORS = 200;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double _powerFactor;
    private double imuTurn;

    public Robot(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime, double powerFactor){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        arm = new Arm(mainHardwareMap, mainTelemetry, elapsedTime, powerFactor);
        clamp = new Clamp(mainHardwareMap);
        imuSensor = new IMUSensor(mainHardwareMap);
        _powerFactor = powerFactor;
    }

    public void Init()
    {
        //Initialize hardware map
        topDrive  = hardwareMap.get(DcMotor.class, "topDrive");
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        arm.Initialize();
        clamp.Initialize();
        imuSensor.Initialize();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void holdArmUp(){
        arm.holdArmUp();
    }

    public void holdArmDown(){
        arm.holdArmDown();
    }

    public void downArm()
    {
        arm.downArm();
    }

    public void stopArm()
    {
        arm.stopArm();
    }

    public void autonomousMoveArmDown(int interval) {
        arm.autonomousMoveArmDown(interval);
    }

    public void CloseOrOpen(boolean userInput)
    {
        clamp.CloseOrOpen(userInput);
    }

    private void setPowerWithPowerFactor(){
        topDrive.setPower(topPower * _powerFactor);
        leftDrive.setPower(leftPower  * _powerFactor);
        rightDrive.setPower(rightPower * _powerFactor);
    }

    public void stopRobot(){
        topPower = 0;
        leftPower = 0;
        rightPower = 0;
        topDrive.setPower(topPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void autonomousMoveForward(int interval){

        imuTurn =  imuSensor.getImuTurn();
        topPower = 0;
        leftPower = -1 + imuTurn;
        rightPower = 1 - imuTurn;
        moveRobot(interval);
    }

    public void autonomousMoveBackward(int interval){
        imuTurn =  imuSensor.getImuTurn();
        topPower = 0;
        leftPower = 1 - imuTurn;
        rightPower = -1 + imuTurn;;
        moveRobot(interval);
    }

    public void autonomousRotate(int interval){
        topPower = 1;
        leftPower = 1 ;
        rightPower = 1;
        moveRobot(interval);
    }

    public void autonomousStrafeLeft(int interval){
        topPower = 1;
        leftPower = 0;
        rightPower = 0;
        moveRobot(interval);
    }

    public void autonomousStrafeRight(int interval){
        topPower = -1;
        leftPower = 0;
        rightPower = 0;
        moveRobot(interval);
    }

    public void moveRobot(int interval)
    {
        setPowerWithPowerFactor();
        wait(interval);
        stopRobot();
    }

    //Autonomous modes ring-0, ring-1, ring-2
    public void AutoRing0() {
        autonomousMoveForward(2200);

        //Drop the wobble
        autonomousMoveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        autonomousMoveBackward(500);
        holdArmUp();
        wait(2000);
        autonomousStrafeRight(600);
        autonomousMoveForward(800);
    }

    public void AutoRing1() {
        //Move Kiwi to the desired block
        autonomousMoveForward(3000);
        autonomousStrafeRight(1150);

        //Drop the wobble
        autonomousMoveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        wait(1000);
        holdArmUp();
        wait(3000);

        //Move Kiwi to the launch zone
        autonomousStrafeLeft(500);
        autonomousMoveBackward(500);
    }

    public void AutoRing4() {
        //Move kiwi to the left
        autonomousStrafeLeft(150);

         //Move Kiwi to the desired block
         autonomousMoveForward(4000);

         //Drop the wobble
        autonomousMoveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        holdArmUp();

        //Move Kiwi to the launch zone
        autonomousMoveBackward(1700);
    }

    public void ForwardOrBackward(double power){
        topPower = 0;
        leftPower = power;
        rightPower = -power;
        SetPower();
    }

    public void Strafe(double power){
        topPower = -power ;
        leftPower = 0;
        rightPower = power;
        SetPower();
    }

    public void Rotate(double power){
        topPower = power;
        leftPower = power;
        rightPower = power;
        SetPower();
    }

    public void StopRobot(){
        topPower = 0;
        leftPower = 0;
        rightPower = 0;
        SetPower();
    }

    private void SetPower(){
        topDrive.setPower(topPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        String logInfo = String.format("top=%.3f, left=%.3f, right=%.3f, imu=%.3f", topPower, leftPower, rightPower, imuTurn);
        log(logInfo);
    }

    public void log(String logInfo)
    {
        telemetry.addLine(logInfo);
        telemetry.update();
    }
}
