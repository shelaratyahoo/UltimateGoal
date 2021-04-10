package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    static final boolean INTAKE_ON = true;
    static final boolean INTAKE_OFF = false;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor innerIntake = null;
    private DcMotor outerIntake = null;
    private final double INTAKE_POWER = 0.65;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double _powerFactor;
    private boolean IntakeState = INTAKE_OFF;

    public Intake(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime, double powerFactor)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        _powerFactor = powerFactor;
    }

    public void Initialize()
    {
        innerIntake = hardwareMap.get(DcMotor.class, "innerIntake");
        outerIntake = hardwareMap.get(DcMotor.class, "outerIntake");

        innerIntake.setDirection(DcMotor.Direction.FORWARD);
        outerIntake.setDirection(DcMotor.Direction.FORWARD);

        innerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void StartOrStop()
    {
        if(IntakeState == INTAKE_OFF)
        {
            Start();
        }
        else
        {
            Stop();
        }
    }

    public void Start()
    {
        innerIntake.setPower(INTAKE_POWER);
        outerIntake.setPower(INTAKE_POWER);
        IntakeState = INTAKE_ON;
    }

    public void Stop()
    {
        innerIntake.setPower(0);
        outerIntake.setPower(0);
        IntakeState = INTAKE_OFF;
    }

    public void InForwardDirection()
    {
        Start();
    }

    public void InReverseDirection()
    {
        innerIntake.setPower(-INTAKE_POWER);
        outerIntake.setPower(-INTAKE_POWER);
        IntakeState = INTAKE_ON;
    }

}
