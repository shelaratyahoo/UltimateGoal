package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static android.os.SystemClock.sleep;

public class LEDClass {
    private ElapsedTime runtime = null;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public LEDClass(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
    }

    public void Initialize(){
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        TurnLedsOff();
    }

    public void wait(int interval){
        sleep(interval);
    }

    public void RedOn()
    {
        redLED.setState(true);
        greenLED.setState(false);
    }

    public void GreenOn()
    {
        redLED.setState(false);
        greenLED.setState(true);
    }

    public boolean IsRedOn()
    {
        return  redLED.getState();
    }

    public boolean IsGreenOn()
    {
        return  greenLED.getState();
    }

    public void TurnLedsOff()
    {
        redLED.setState(true);
        greenLED.setState(true);
    }

    public void TurnLedsOn()
    {
        redLED.setState(false);
        greenLED.setState(false);
    }
}

