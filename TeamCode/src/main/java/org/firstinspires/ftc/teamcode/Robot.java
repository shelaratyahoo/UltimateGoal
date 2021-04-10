package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_POWERSHOT_1_SPEED;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_POWERSHOT_2_SPEED;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_POWERSHOT_3_SPEED;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_TOPLEVEL_1_SPEED;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_TOPLEVEL_2_SPEED;
import static org.firstinspires.ftc.teamcode.Shooter.SHOOTER_RING_TOPLEVEL_3_SPEED;

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
        arm.Initialize();
        clamp.Initialize();
        shooter.Initialize();
        tipper.Initialize();
        intake.Initialize();
        t265.Initialize();
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void StartT265() { t265.Start();}
    public void StopT265() { t265.Stop();}
    public void GetRobotPosition() { t265.GetXYPosition();}

    public void ArmUp(){
        arm.Up();
    }

    public void ArmDown(){
        arm.Down();
    }

    public void AutoArmUp(){
        arm.Up(5900);
        arm.stop();
    }

    public void AutoArmDown(){
        arm.Down(400);
        arm.stop();
    }

    public void stopArm()
    {
        arm.stopArm();
    }

    public void autonomousMoveArmDown(int interval) {
        arm.autonomousMoveArmDown(interval);
    }

    public void OpenOrCloseClamp()
    {
        clamp.OpenOrClose();
    }

    private void setPowerWithPowerFactor(){
        chassis.setPowerWithPowerFactor();
    }

    public void stopRobot(){
        chassis.stop();
    }

    public void PushOrPullTipper(){ tipper.PushOrPull(true);}

    public void autonomousMoveForward(int interval){
        chassis.moveForwardOrBackward(-autonomousPower);
        wait(interval);
        chassis.stop();
        wait(100);
    }

    public void autonomousMoveForwardWithImu(int interval){
        chassis.moveForwardOrBackwardWithImu(-autonomousPower);
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
        chassis.RotateWithFullPower(-autonomousPower);
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

    public void moveRobot(int interval)
    {
        chassis.setPowerWithPowerFactor();
        wait(interval);
        chassis.stop();
    }

    //Autonomous modes ring-0, ring-1, ring-2
    public void AutoRing0() {
        AutoCommon();

        //Reset TriForce position
        autonomousRotateAntiClockwise(400);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisUp(1000);
        wait(400);

        //Lower down the arm.
        arm.Down(3600);

        //Drop the wobble
        clamp.OpenOrClose();

        //Move TriForce to launch zone
        autonomousLeftAxisDown(100);
        wait(400);
    }

    public void AutoRing1()
    {
        AutoCommon();

        //Reset TriForce position
        autonomousRotateAntiClockwise(700);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisUp(1000);
        wait(400);

        //Lower down the arm.
        arm.Down(3600);

        //Drop the wobble
        clamp.OpenOrClose();

        //Move TriForce to launch zone
        autonomousLeftAxisDown(400);

    }

    public void AutoCommon()
    {
        //Move TriForce to the launch zone.
        autonomousRotateClockwise(45);
        wait(500);
        autonomousMoveForwardWithImu(1300);
        wait(4000);

        //Fire at 1st power shot
        shooter.FireRing(SHOOTER_RING_POWERSHOT_2_SPEED);
        autonomousRotateClockwise(5);
        wait(2000);
        shooter.FireRing(SHOOTER_RING_POWERSHOT_3_SPEED);
        autonomousRotateClockwise(7);
        wait(1500);
        shooter.FireRing(SHOOTER_RING_POWERSHOT_1_SPEED);

        //Turn OFF the shooter.
        shooter.Stop();
        wait(100);
    }

    public void AutoRing4() {
        AutoCommon();

        //Reset TriForce position
        autonomousRotateAntiClockwise(600);
        wait(400);

        //Move TriForce to the C drop zone
        autonomousLeftAxisUp(1900);
        wait(400);

        //Lower down the arm.
        arm.Down(3600);

        //Drop the wobble
        clamp.OpenOrClose();

        //Move TriForce to launch zone
        autonomousLeftAxisDown(1800);

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
