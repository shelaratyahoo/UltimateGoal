package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

public class Shooter {
    private DcMotor shooter = null;
    private Servo feeder  = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FEEDER_UP = 0;     // Clamp open position
    static final double FEEDER_DOWN = .5;    // Clamp close position
    static final double SHOOTER_RING_1_SPEED = -0.4; //Shooter initialize speed.
    static final double SHOOTER_RING_2_SPEED = -0.5; //Shooter initialize speed.
    static final double SHOOTER_RING_3_SPEED = -0.54; //Shooter initialize speed.
    static final double SHOOTER_AUTO_FIRING = -0.45; //Shooter initialize speed.

    private double shooterPower = 0;

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
        shooter  = hardwareMap.get(DcMotor.class, "shooter");
        feeder = hardwareMap.get(Servo.class, "feeder");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        feeder.setPosition(FEEDER_DOWN);
        shooterPower = SHOOTER_RING_1_SPEED;
    }

    public void fireRingsAtPowerShot()
    {
        //Fire 1st ring
        feeder.setPosition(FEEDER_UP);
        sleep(100);

        //Positioned TriForce at 2nd power shot

    }
    public void fireRings()
    {
        fireRing(SHOOTER_RING_2_SPEED);
        fireRing(SHOOTER_RING_3_SPEED);
        fireRing(SHOOTER_RING_1_SPEED);
    }

    public void fireRing(double newPower)
    {
        feeder.setPosition(FEEDER_UP);
        sleep(150);
        shooter.setPower(newPower);
//        telemetry.addData("Shooter", "left (%.2f)", newPower);
//        telemetry.update();
        feeder.setPosition(FEEDER_DOWN);
        sleep(620);
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void startShooter(){
        shooter.setPower(-0.48);
    }
    public void stopShooter(){ shooter.setPower(0); }

    public void stop(){ shooter.setPower(0); }

}

