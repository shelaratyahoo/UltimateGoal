package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

public class Robot {
    static final double MOVE_SERVO = 1;
    static final double STOP_SERVO = 0.5;
    static final double OPEN_CLAMP = 1;     // Clamp open position
    static final double CLOSE_CLAMP = 0;    // Clamp close position
    static final int    SLEEP_FOR_SERVO_MOTORS = 200;
    static final double SERVO_STOP = 0.5;
    static final double SERVO_ROTATE_CLOCKWISE = 1;
    static final double SERVO_ROTATE_ANTI_CLOCKWISE = 0;


    private ElapsedTime runtime = new ElapsedTime();
    private Chassis chassis = null;
    private Clamp clamp = null;
    private Arm arm = null;
    private Shooter shooter = null;
    private Tipper tipper = null;
    private Intake intake = null;
    private T265 t265 = null;

    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;
    double autonomousPower = 1;

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
        tipper = new Tipper(mainHardwareMap);
        intake = new Intake(mainHardwareMap, mainTelemetry, elapsedTime, powerFactor);
        t265 = new T265(mainHardwareMap, mainTelemetry, elapsedTime);

        _powerFactor = powerFactor;
        autonomousPower = _powerFactor * 1;
    }

    public void Init()
    {
        //Initialize hardware map
        chassis.Initialize();
        clamp.Initialize();
        arm.Initialize();
        shooter.Initialize();
        tipper.Initialize();
        intake.Initialize();
        t265.Initialize();
    }

    public void InitAuto()
    {
        //Initialize hardware map
        chassis.InitializeAuto();
        clamp.Initialize();
        arm.Initialize();
        shooter.Initialize();
        tipper.Initialize();
        intake.Initialize();
        t265.Initialize();
        arm.UpOrDown();
        arm.Stop();
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void StartT265() { t265.Start();}
    public void StopT265() { t265.Stop();}
    public void GetRobotPosition() { t265.GetXYPosition();}

    public void ArmUpOrDown(){ arm.UpOrDown(); }
    public void ArmDown() { arm.Down(); }
    public void ArmForceDown() {arm.ForceDown();}
    public void ArmStop() { arm.Stop();}
    public void OpenOrCloseClamp()
    {
        clamp.OpenOrClose();
    }
    public void PushOrPullTipper(){ tipper.PushOrPull(true);}

    public void autonomousMoveForward(int interval){
        chassis.moveForwardOrBackward(-autonomousPower, interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousMoveBackward(int interval){
        chassis.moveForwardOrBackward(-autonomousPower, interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRotate(int interval){
        chassis.rotate(autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRotateClockwise(int interval){
        chassis.RotateWithFullPower(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRotateAntiClockwise(int interval){
        chassis.rotate(autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousLeftAxisForward(int interval){
        chassis.moveLeftAxis(-autonomousPower, interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousLeftAxisBackward(int interval){
        chassis.moveLeftAxis(autonomousPower, interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRightAxisForward(int interval){
        chassis.moveRightAxis(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousRightAxisBackward(int interval){
        chassis.moveRightAxis(autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousStrafeLeft(int interval){
        chassis.strafeLeftOrRight(autonomousPower);
        wait(interval);

        chassis.strafeLeftOrRight(0.3);
        wait(200);

        chassis.stop();
        wait(100);
    }

    public void autonomousStrafeRight(int interval){
        chassis.strafeLeftOrRight(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    //Autonomous modes ring-0, ring-1, ring-2
    public void AutoRing0_BluePowerShots() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(550);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(1500);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(300);
        autonomousRotateAntiClockwise(300);
        autonomousLeftAxisForward(300);
        autonomousMoveForward(400);
    }

    public void AutoRing0_BlueTowerTop() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(800);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(900);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(300);
        wait(100);
        autonomousRotateAntiClockwise(300);
        wait(100);
        autonomousLeftAxisForward(300);
        wait(100);
        autonomousRightAxisBackward(800);
        autonomousLeftAxisForward(700);
        autonomousRightAxisBackward(800);
    }

    public void AutoRing1_BluePowerShots() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(770);
        wait(400);

        //Move TriForce to the B drop zone
        autonomousLeftAxisForward(1500);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(500);
    }

    public void AutoRing1_BlueTowerTop() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(1000);
        wait(400);

        //Move TriForce to the B drop zone
        autonomousLeftAxisForward(2200);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(500);
    }

    public void AutoRing4_BluePowerShots() {

        //Reset TriForce position
        autonomousRotateAntiClockwise(730);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(2280);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(1500);
    }

    public void AutoRing4_BlueTowerTop() {

        //Reset TriForce position
        autonomousRotateAntiClockwise(900);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(600);
        wait(100);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(60);
        autonomousMoveForward(1800);
        autonomousLeftAxisBackward(600);
    }

    public void CpRing0() {
//
//        //Reset TriForce position
//        autonomousRotateAntiClockwise(400);
//        wait(400);
//
//        //Move TriForce to the C drop zone
//        autonomousLeftAxisUp(1000);
//        wait(400);

        //Lower down the arm.
        arm.UpOrDown();

        //Drop the wobble
        clamp.OpenOrClose();

//        //Move TriForce to launch zone
//        autonomousLeftAxisDown(100);
//        wait(400);
    }

    public void AutoRing0_RedPowerShots() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(1200);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(1500);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(300);
        autonomousRotateAntiClockwise(300);
        autonomousLeftAxisForward(300);
        autonomousMoveForward(400);
    }

    public void AutoRing0_RedTowerTop() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(800);//verify the rotation

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(900);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(300);
        wait(100);
        autonomousRotateAntiClockwise(300);
        wait(100);
        autonomousLeftAxisForward(300);
        wait(100);
        autonomousRightAxisBackward(800);
        autonomousLeftAxisForward(700);
        autonomousRightAxisBackward(800);
    }

    public void AutoRing1_RedPowerShots() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(770);
        wait(400);

        //Move TriForce to the B drop zone
        autonomousLeftAxisForward(1500);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(500);
    }

    public void AutoRing1_RedTowerTop() {
        //Reset TriForce position
        autonomousRotateAntiClockwise(500);//verify the rotation
        wait(400);

        //Move TriForce to the B drop zone
        autonomousLeftAxisForward(2200);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(500);
    }

    public void AutoRing4_RedPowerShots() {

        //Reset TriForce position
        autonomousRotateAntiClockwise(950);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(2280);
        wait(400);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(1500);
    }

    public void AutoRing4_RedTowerTop() {

        //Reset TriForce position
        autonomousRotateAntiClockwise(800);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(1000);
        autonomousRotateAntiClockwise(10);
        autonomousLeftAxisForward(600);
        wait(100);

        //Lower down the arm.
        arm.UpOrDown();
        wait(200);

        //Drop the wobble
        clamp.OpenOrClose();
        wait(200);

        //Move TriForce to launch zone
        autonomousLeftAxisBackward(60);
        autonomousMoveBackward(1800);
        autonomousLeftAxisBackward(600);
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

    public void StartOrStopShooter()
    {
        shooter.StartOrStop();
    }

    public void FireRingsAtPowerShot()
    {
        shooter.FireRingsAtPowerShot();
    }

    public void FireRingsAtTopLevel()
    {
        shooter.FireRingsAtTopLevel();
    }

    public void StartOrStopIntake()
    {
        intake.StartOrStop();
    }

    public void StopIntake()
    {
        intake.Stop();
    }

    public void RotateIntakeInReverseDirection() { intake.InReverseDirection();}

    public void RotateIntakeInForwardDirection() { intake.InForwardDirection();}

}
