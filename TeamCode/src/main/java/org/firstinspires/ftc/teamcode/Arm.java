package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

public class Arm {
    private DcMotor arm = null;
    private ElapsedTime runtime = new ElapsedTime();

    private double armPower = 0;

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
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void holdArmUp(){
        arm.setPower(.8);
        wait(50);
    }

    public void holdArmDown(){
        arm.setPower(-0.2);
    }

    public void downArm()
    {
        arm.setPower(-0.2);
        wait(500);
        stopArm();
        wait(1);
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

}
