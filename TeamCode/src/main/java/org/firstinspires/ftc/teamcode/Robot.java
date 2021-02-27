package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private Chassis chassis = null;
    private Clamp clamp = null;
    private Arm arm = null;

    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;
    double autonomousPower = 1;

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

        chassis = new Chassis(mainHardwareMap, mainTelemetry, elapsedTime, powerFactor);
        arm = new Arm(mainHardwareMap, mainTelemetry, elapsedTime, powerFactor);
        clamp = new Clamp(mainHardwareMap);
        _powerFactor = powerFactor;
    }

    public void Init()
    {
        //Initialize hardware map
        chassis.Initialize();
        arm.Initialize();
        clamp.Initialize();
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
        chassis.setPowerWithPowerFactor();
    }

    public void stopRobot(){
        chassis.stop();
    }

    public void autonomousMoveForward(int interval){

        chassis.moveForwardOrBackward(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousMoveBackward(int interval){
        chassis.moveForwardOrBackward(-autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRotate(int interval){
        chassis.rotate(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousLeftAxis(int interval){
        chassis.moveLeftAxis(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRightAxis(int interval){
        chassis.moveRightAxis(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousLeft(int interval){
        chassis.strafeLeftOrRight(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRight(int interval){
        chassis.strafeLeftOrRight(-autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void moveRobot(int interval)
    {
        chassis.setPowerWithPowerFactor();
        wait(interval);
        chassis.stop();
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
        autonomousRightAxis(600);
        autonomousMoveForward(800);
    }

    public void AutoRing1() {
        //Move Kiwi to the desired block
        autonomousMoveForward(3000);
        autonomousRightAxis(1150);

        //Drop the wobble
        autonomousMoveArmDown(50);
        wait(2000);
        clamp.CloseOrOpen(true);
        wait(1000);
        holdArmUp();
        wait(3000);

        //Move Kiwi to the launch zone
        autonomousLeftAxis(500);
        autonomousMoveBackward(500);
    }

    public void AutoRing4() {
        //Move kiwi to the left
        autonomousLeftAxis(150);

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
        chassis.moveForwardOrBackward(power);
    }

    public void StrafeLeftOrRight(double power){
        chassis.strafeLeftOrRight(power);
    }

    public void Rotate(double power){
        chassis.rotate(power);
    }

    public void MoveLeftAxis(double power){
        chassis.moveLeftAxis(power);
    }

    public void MoveRightAxis(double power){
        chassis.moveRightAxis(power);
    }

    public void StopRobot(){
        chassis.stop();
    }

}
