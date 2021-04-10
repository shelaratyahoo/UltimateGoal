package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

public class Shooter {
    private DcMotorEx shooter = null;
    private Servo feeder  = null;
    private ElapsedTime runtime = new ElapsedTime();

    public static final double FEEDER_UP = 0;     // Clamp open position
    public static final double FEEDER_DOWN = .5;    // Clamp close position

    public static final double SHOOTER_RPM = 1000;
    public static final double SHOOTER_STOP = 0;
    public static final double SHOOTER_MAX_RPM = 1800;

    static final double SHOOTER_RING_POWERSHOT_1_SPEED = -0.37;
    static final double SHOOTER_RING_POWERSHOT_2_SPEED = -0.40;
    static final double SHOOTER_RING_POWERSHOT_3_SPEED = -0.47;

    static final double SHOOTER_RING_TOPLEVEL_1_SPEED = -0.38;
    static final double SHOOTER_RING_TOPLEVEL_2_SPEED = -0.42;
    static final double SHOOTER_RING_TOPLEVEL_3_SPEED = -0.49;
    static final double SHOOTER_AUTO_FIRING = -0.45;

    static final boolean SHOOTER_ON = true;
    static final boolean SHOOTER_OFF = false;

    private double shooterPower = 0;
    private boolean shooterState = SHOOTER_OFF;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double _powerFactor;

    public Shooter(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime, double powerFactor)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        _powerFactor = powerFactor;
    }

    public void Initialize(){
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.get(Servo.class, "feeder");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setPosition(FEEDER_DOWN);
        shooterPower = SHOOTER_RING_POWERSHOT_1_SPEED;
        shooterState = SHOOTER_OFF;
    }

    public void StartOrStop()
    {
        if(shooterState == SHOOTER_OFF)
        {
            Start();
        }
        else
        {
            Stop();
        }
    }

    public void FireRingsAtPowerShot()
    {
        if(shooterState == SHOOTER_ON)
        {
            FireRing(SHOOTER_RING_POWERSHOT_2_SPEED);
            FireRing(SHOOTER_RING_POWERSHOT_3_SPEED);
            FireRing(SHOOTER_RING_POWERSHOT_1_SPEED);
        }
    }

    public void FireRingsAtTopLevel()
    {
        if(shooterState == SHOOTER_ON)
        {
            FireRing();
            FireRing();
            FireRing();
        }
    }

    public void FireRing(double newPower)
    {
        feeder.setPosition(FEEDER_UP);
        sleep(150);
        shooter.setPower(newPower);
        feeder.setPosition(FEEDER_DOWN);
        sleep(620);
    }

    public void FireRing()
    {
        double currentVelocity = shooter.getVelocity();
        runtime.reset();
        while(currentVelocity < (SHOOTER_RPM - 10) || currentVelocity > (SHOOTER_RPM + 20))
        {
            //Set the shooter velocity to SHOOTER_RPM
            shooter.setVelocity(SHOOTER_RPM);
            sleep(200);
            currentVelocity = shooter.getVelocity();

            //wait for 5 seconds to fire the ring, else bail out.
            if(runtime.seconds() > 5) {
                telemetry.addData("Bailed out of the FireRing loop","move on...");
                telemetry.update();
                return;
            }
        }
        feeder.setPosition(FEEDER_UP);
        sleep(200);
        feeder.setPosition(FEEDER_DOWN);
        sleep(200);
    }

    public void FeederTesting()
    {
        feeder.setPosition(FEEDER_UP);
        sleep(150);
        feeder.setPosition(FEEDER_DOWN);
        sleep(150);
    }

    public void wait(int interval){ sleep(interval); }
    public void Start(){  SetVelocity(SHOOTER_RPM);  shooterState = SHOOTER_ON; }
    public void Start(double setPower){  shooter.setPower(setPower);  shooterState = SHOOTER_ON; }
    public void Stop(){ SetVelocity(SHOOTER_STOP); shooterState = SHOOTER_OFF; }
    public double GetVelocity() { return shooter.getVelocity();}
    public void SetVelocity(double angularRate) { shooter.setVelocity(angularRate);}
    public boolean GetShooterState() { return shooterState;}

    public void SetVelocityPIDFCoefficients(double p, double i, double d, double f) {
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void SetPositionPIDFCoefficients(double p) {
        shooter.setPositionPIDFCoefficients(p);
    }
}

