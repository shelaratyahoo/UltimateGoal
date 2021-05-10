package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static final double SHOOTER_RPM = 1000;
    public static final double SHOOTER_STOP = 0;
    public static final double SHOOTER_MAX_RPM = 1940; //1800;
    public static final double SHOOTER_TOPLEVEL_RPM = 1000;
    public static final double SHOOTER_POWERSHOT_RPM = 1000;

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
        shooterPIDCoefficient = new PIDFCoefficients();
        shooterPIDCoefficient.p = 100;//2500;
        shooterPIDCoefficient.i = 0.1;//0.1f;
        shooterPIDCoefficient.d = 0.001;//10;
        shooterPIDCoefficient.f = 17;//900;
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

        double F = 0f;//32767 / SHOOTER_MAX_RPM ;
        double P = 0f;//F * 0.1;
        double I = 0f;//P * 0.1;
        double D = 0f;
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDCoefficient);
        shooter.setVelocityPIDFCoefficients(shooterPIDCoefficient.p, shooterPIDCoefficient.i, shooterPIDCoefficient.d, shooterPIDCoefficient.f);
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
            FireRingV1(SHOOTER_POWERSHOT_RPM, true);
        }
    }

    public void FireRingsAtTopLevel()
    {
        if(shooterState == SHOOTER_ON)
        {
            FireRingV1(SHOOTER_TOPLEVEL_RPM, false);
            FireRingV1(SHOOTER_TOPLEVEL_RPM, false);
            FireRingV1(SHOOTER_TOPLEVEL_RPM, false);
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

    public void FireRingV1(double targetVelocity, boolean isSingleFire)
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
        if(isSingleFire)
        {
            sleep(200);
        }
        else
        {
            sleep(200);
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

