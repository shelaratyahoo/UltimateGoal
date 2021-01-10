package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private Servo clamp = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Robot(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;

    }

    public void moveArm() {

    }

}
