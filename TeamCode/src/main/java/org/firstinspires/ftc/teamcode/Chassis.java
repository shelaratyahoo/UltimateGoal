package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

public class Chassis {
    private DcMotor topDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private IMUSensor imuSensor = null;

    double leftPower = 0;
    double rightPower = 0;
    double topPower = 0;
    private double _powerFactor;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime;

    private double imuTurn;

    static final boolean DIRECTION_CLOCKWISE = true;
    static final boolean DIRECTION_COUNTERCLOCKWISE = false;

    public Chassis(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime, double powerFactor)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        _powerFactor = powerFactor;
        imuSensor = new IMUSensor(mainHardwareMap);
    }

    public void Initialize ()
    {
        //Initialize hardware map
        topDrive  = hardwareMap.get(DcMotor.class, "topDrive");
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        //Set mode run using encoder
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Zero power Behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize the IMU Sensor
        imuSensor.Initialize();
    }

    public void moveForwardOrBackward(double power)
    {
        imuTurn = imuSensor.getImuTurn();
        topPower = 0;
        leftPower = -power;
        rightPower = power;
        setPower();
    }

    public void strafeLeftOrRight(double power)
    {
        imuTurn = imuSensor.getImuTurn();
        topPower = power;
        leftPower = -power/2;
        rightPower = power/2;
        setPower();
    }

    public void moveLeftAxis(double power)
    {
        imuTurn = imuSensor.getImuTurn();
        topPower = power;
        leftPower = 0;
        rightPower = -power;
        setPower();
    }

    public void moveRightAxis(double power)
    {
        imuTurn = imuSensor.getImuTurn();
        topPower = -power;
        leftPower = power;
        rightPower = 0;
        setPower();
    }

    public void rotate(double power)
    {
        imuTurn = imuSensor.getImuTurn();
        topPower = power;
        leftPower = power;
        rightPower = power;
        setPower();
    }

    public void stop(){
        topPower = 0;
        leftPower = 0;
        rightPower = 0;
        setPower();
    }

    public void setPowerWithPowerFactor(){
        topDrive.setPower(topPower * _powerFactor);
        leftDrive.setPower(leftPower  * _powerFactor);
        rightDrive.setPower(rightPower * _powerFactor);
        String logInfo = String.format("top=%.3f, left=%.3f, right=%.3f, imu=%.3f", topPower, leftPower, rightPower, imuTurn);
        log(logInfo);
    }

    private void setPower(){
        topDrive.setPower(topPower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        String logInfo = String.format("top=%.3f, left=%.3f, right=%.3f, imu=%.3f", topPower, leftPower, rightPower, imuTurn);
        log(logInfo);
    }

    private void wait(int interval){
        sleep(interval);
    }

    public void log(String logInfo)
    {
        telemetry.addLine(logInfo);
        telemetry.update();
    }
}
