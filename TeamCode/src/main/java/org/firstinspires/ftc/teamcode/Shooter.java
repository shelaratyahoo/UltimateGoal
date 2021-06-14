package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private ElapsedTime pidTimer = new ElapsedTime();
    private PIDFCoefficients shooterPIDCoefficient = null;

    public static final double FEEDER_UP = 0;     // Clamp open position
    public static final double FEEDER_DOWN = .5;    // Clamp close position

    public static final double SHOOTER_TOWERTOP_RPM = 640;
    public static final double SHOOTER_POWERSHOT_RPM = 560;//1000;
    public static final double SHOOTER_RPM = 520; //1000;
    public static final double SHOOTER_STOP = 0;
    public static final double SHOOTER_MAX_RPM = 1940; //1800;

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
        shooterPIDCoefficient = new PIDFCoefficients();
        shooterPIDCoefficient.p = 100; //100;//2500;
        shooterPIDCoefficient.i = 0; //0.1;//0.1f;
        shooterPIDCoefficient.d = 0; //0.001;//10;
        shooterPIDCoefficient.f = 17; //17;//900;
    }

    public void Initialize(){
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.get(Servo.class, "feeder");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder.setPosition(FEEDER_DOWN);
        shooterPower = -0.4; // SHOOTER_RING_POWERSHOT_1_SPEED;
        shooterState = SHOOTER_OFF;

        shooter.setVelocityPIDFCoefficients(shooterPIDCoefficient.p, shooterPIDCoefficient.i, shooterPIDCoefficient.d, shooterPIDCoefficient.f);
    }

    public boolean IsShooterOn() { return (shooterState == SHOOTER_ON); }

    public boolean IsShooterOff() { return (shooterState == SHOOTER_OFF); }

    public void StartOrStop(boolean towerTop)
    {
        if(shooterState == SHOOTER_OFF)
        {
            if(towerTop)
            {
                StartShootingTowerTop();
            }
            else
            {
                StartShootingPowerShot();
            }
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
            FireRingV1(SHOOTER_POWERSHOT_RPM, true);
        }
    }

    public void FireRingsAtTopLevel()
    {
        if(shooterState == SHOOTER_ON)
        {
            FireRingV1(SHOOTER_TOWERTOP_RPM, false);
            wait(500);
            FireRingV1(SHOOTER_TOWERTOP_RPM, true);
            wait(500);
            FireRingV1(SHOOTER_TOWERTOP_RPM, false);
        }
    }

    public void FireRingsAtTopLevelAuto()
    {
        Start();
        FireRingsAtTopLevel();
        Stop();
    }

    public void FireRing(double newPower)
    {
        feeder.setPosition(FEEDER_UP);
        sleep(150);
        shooter.setPower(newPower);
        feeder.setPosition(FEEDER_DOWN);
        sleep(620);
    }

    public void FireRingV1(double targetVelocity, boolean is1stTime)
    {
        double currentVelocity = shooter.getVelocity();
        double error = currentVelocity - targetVelocity;
        double lastError = 0;
        double integral = 0;
        pidTimer.reset();
        runtime.reset();
        while(Math.abs(error) != 0)
        {
            currentVelocity = shooter.getVelocity();
            error = currentVelocity - targetVelocity;
            telemetry.addData("Current Velocity", "%.2f", currentVelocity);
            if(runtime.seconds() > 5) {
                telemetry.addData("Bailed out of the FireRing loop","move on...");
                telemetry.update();
                return;
            }
            telemetry.update();
        }

//        while(Math.abs(error) > 7)
//        {
//            double pidTime = pidTimer.time();
//            double currentVelocity = shooter.getVelocity();
//            error = currentVelocity - targetVelocity;
//            double changeInError = lastError - error;
//            integral += changeInError * pidTime;
//            double derivative = changeInError / pidTime;
//            shooterPIDCoefficient.p *= error;
//            shooterPIDCoefficient.i *= integral;
//            shooterPIDCoefficient.d *= derivative;
//            shooter.setVelocityPIDFCoefficients(shooterPIDCoefficient.p, shooterPIDCoefficient.i, shooterPIDCoefficient.d, shooterPIDCoefficient.f);
//            currentVelocity = shooter.getVelocity();
//            telemetry.addData("Setting Velocity", "%.2f", currentVelocity);
//            telemetry.update();
//            lastError = error;
//            pidTimer.reset();
//            if(runtime.seconds() > 10) {
//                telemetry.addData("Current Velocity", "%.2f", currentVelocity);
//                telemetry.addData("Bailed out of the FireRing loop","move on...");
//                telemetry.update();
//                return;
//            }
//        }
        feeder.setPosition(FEEDER_UP);
        telemetry.addData("----- Shooting Velocity -----", "%.2f", shooter.getVelocity());
        telemetry.update();
        if(is1stTime)
        {
            sleep(120);
        }
        else
        {
            sleep(70);
        }
        feeder.setPosition(FEEDER_DOWN);
        sleep(175);
    }

    public void FireRingV2(double targetVelocity)
    {
        double currentVelocity = shooter.getVelocity();
        double error = currentVelocity - targetVelocity;
        double lastError = 0;
        double integral = 0;
        pidTimer.reset();
        runtime.reset();
        while(Math.abs(error) == 0)
        //while(lastError < 1)
        {
            currentVelocity = shooter.getVelocity();
            error = currentVelocity - targetVelocity;
            telemetry.addData("Current Velocity", "%.2f", currentVelocity);
            if(runtime.seconds() > 5) {
                telemetry.addData("Bailed out of the FireRing loop","move on...");
                telemetry.update();
                return;
            }
            telemetry.update();
        }

        feeder.setPosition(FEEDER_UP);
        sleep(175);
        feeder.setPosition(FEEDER_DOWN);
        sleep(175);
        telemetry.addData("Shooting Velocity...", "%.2f", shooter.getVelocity());
        telemetry.update();
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
    public void StartShootingPowerShot(){ SetVelocity(SHOOTER_POWERSHOT_RPM);  shooterState = SHOOTER_ON; }
    public void StartShootingTowerTop(){ SetVelocity(SHOOTER_TOWERTOP_RPM);  shooterState = SHOOTER_ON; }
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

