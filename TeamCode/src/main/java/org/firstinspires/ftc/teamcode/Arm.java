package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


public class Arm {
    private DcMotorEx arm = null;
    private ElapsedTime runtime = null;

    public static final double ARM_START = 1;
    public static final double ARM_STOP = 0;
    public static final int ARM_MAX_POS = 6000; //5600
    public static final int ARM_MIN_POS = 0;
    public static final  double SET_POSITION_PIDF_COEFFICIENTS = 7.1;
    public static final boolean ARM_UP = true;
    public static final boolean ARM_DOWN = false;

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
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(ARM_MIN_POS);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPositionPIDFCoefficients(SET_POSITION_PIDF_COEFFICIENTS);
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void UpOrDown()
    {
        int currentPosition = arm.getCurrentPosition();
        if(currentPosition <= ARM_MIN_POS )
        {
            UpOrDown(currentPosition, ARM_UP);
        }
        else if(currentPosition > ARM_MIN_POS )
        {
            UpOrDown(currentPosition, ARM_DOWN);
        }
        arm.setPower(ARM_STOP);
    }

    public void UpOrDown(int currentPosition, boolean goUp)
    {
        int setTargetPosition = goUp ? ARM_MAX_POS : ARM_MIN_POS;

        SetTargetPosition(setTargetPosition);
        runtime.reset();
        while((goUp && currentPosition < setTargetPosition ) ||
              (!goUp && currentPosition > setTargetPosition ))
        {
            Start();
            if(arm.isBusy()) {
                wait(200);
            }
            if(runtime.seconds() > 5)
            {
                break;
            }
            currentPosition = GetCurrentPosition();
        }
        Stop();
    }

    public void Up()
    {
        int currentPosition = GetCurrentPosition();
        SetTargetPosition(ARM_MAX_POS);
        runtime.reset();
        while(currentPosition < ARM_MAX_POS )
        {
            Start();
            if(arm.isBusy()) {
                wait(200);
            }
            if(runtime.seconds() > 5)
            {
                break;
            }
            currentPosition = GetCurrentPosition();
        }
        Stop();
    }

    public void Down()
    {
        int currentPosition = GetCurrentPosition();
        SetTargetPosition(ARM_MIN_POS);
        runtime.reset();
        while(currentPosition > ARM_MIN_POS )
        {
            Start();
            if(arm.isBusy()) {
                wait(200);
            }
            if(runtime.seconds() > 5)
            {
                break;
            }
            currentPosition = GetCurrentPosition();
        }
        Stop();
    }

    public void ForceDown()
    {
        arm.setTargetPosition(ARM_MAX_POS);
        int currentPosition = GetCurrentPosition();
        runtime.reset();
        SetTargetPosition(-ARM_MAX_POS);
        while(currentPosition > (-ARM_MAX_POS) )
        {
            Start();
            if(arm.isBusy()) {
                wait(200);
            }
            if(runtime.seconds() > 5)
            {
                break;
            }
            currentPosition = GetCurrentPosition();
        }
        Stop();
        arm.setTargetPosition(ARM_MIN_POS);
    }

    public int GetCurrentPosition()
    {
        return arm.getCurrentPosition();
    }

    public void SetTargetPosition(int position)
    {
        arm.setTargetPosition(position);
    }

    public void Start(){ arm.setPower(ARM_START); }
    public void Stop(){ arm.setPower(ARM_STOP); }

    public void DPadDown()
    {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(-1);
        wait(100);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void DPadUp()
    {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(1);
        wait(100);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

