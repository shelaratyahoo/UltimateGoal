package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


public class Arm {
    private DcMotor arm = null;
    private ElapsedTime runtime = new ElapsedTime();

    public static final boolean ARM_UP = true;
    public static final boolean ARM_DOWN = false;

    private double armPower = 0;
    private boolean position = ARM_UP;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double _powerFactor;

    public Arm(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime, double powerFactor)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        _powerFactor = powerFactor;
    }

    public void Initialize(){
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotor.Direction.FORWARD);
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void holdArmUp(){
//        arm.setPower(.8);
//        wait(50);
    }

    public void holdArmDown(){
//        arm.setPower(-0.2);
    }

    public void downArm()
    {
//        arm.setPower(-0.2);
//        wait(500);
//        stopArm();
//        wait(1);
    }

    public void Down()
    {
        Down(500);
    }

    public void Down(int interval)
    {
        arm.setPower(1);
        wait(interval);
    }

    public void Up()
    {
        Up(500);
    }

    public void Up(int interval)
    {
        arm.setPower(-1);
        wait(interval);
    }

    public void autonomousMoveArmDown(int interval) {
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

    public void startArm(){
        arm.setPower(armPower * _powerFactor);
    }

    public void stopArm(){ arm.setPower(0); }
    public void stop(){ arm.setPower(0); }

}

