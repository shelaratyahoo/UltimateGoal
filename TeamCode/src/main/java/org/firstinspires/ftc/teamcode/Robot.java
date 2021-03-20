package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private Chassis chassis = null;
    private Clamp clamp = null;
    private Arm arm = null;
    private Shooter shooter = null;

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
        shooter = new Shooter(mainHardwareMap, mainTelemetry, elapsedTime, powerFactor);
        clamp = new Clamp(mainHardwareMap);
        _powerFactor = powerFactor;
        autonomousPower = _powerFactor * 1;
    }

    public void Init()
    {
        //Initialize hardware map
        chassis.Initialize();
        arm.Initialize();
        clamp.Initialize();
        shooter.Initialize();
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void holdArmUp(){
        arm.holdArmUp();
    }

    public void ArmUp(){
        arm.Up(5900);
        arm.stop();
    }

    public void ArmDown(){
        arm.Down(400);
        arm.stop();
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

        chassis.moveForwardOrBackward(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousMoveBackward(int interval){
        chassis.moveForwardOrBackward(autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRotate(int interval){
        chassis.rotate(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRotateClockwise(int interval){
        chassis.rotate(-autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRotateAntiClockwise(int interval){
        chassis.rotate(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousLeftAxisUp(int interval){
        chassis.moveLeftAxis(-autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousLeftAxisDown(int interval){
        chassis.moveLeftAxis(autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousRightAxis(int interval){
        chassis.moveRightAxis(-autonomousPower);
        wait(interval);
        chassis.stop();
    }

    public void autonomousStrafeLeft(int interval){
        chassis.strafeLeftOrRight(autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousStrafeRight(int interval){
        chassis.strafeLeftOrRight(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
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
        autonomousLeftAxisUp(500);
        autonomousMoveBackward(500);
    }

    public void AutoRing4() {

        //Turn ON the shooter.
        shooter.startShooter();

        //Move TriForce to the launch zone.
        autonomousMoveForward(2800);
        wait(150);

        autonomousRotateClockwise(1);
        wait(250);

        //Fire at 1st power shot
        shooter.fireRing(-0.42);

        //Positioned TriForce to the 2nd ring
        autonomousRotateClockwise(1);
        wait(150);

        //Fire at 2nd power shot
        shooter.fireRing(-0.42);

        //Positioned TriForce to the 3rd ring
        autonomousRotateClockwise(1);
        wait(100);

        //Fire at 3rd power shot
        shooter.fireRing(Shooter.SHOOTER_AUTO_FIRING);

        //Turn OFF the shooter.
        shooter.stopShooter();
        wait(100);

        //Reset TriForce position
        autonomousRotateClockwise(700);
        wait(100);

        //Move TriForce to the C drop zone
        autonomousLeftAxisUp(4000);

        //Lower down the arm.
        arm.Down(3500);

        //Drop the wobble
        clamp.CloseOrOpen(true);

        //Move TriForce to launch zone
        autonomousLeftAxisDown(2700);
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
