package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static android.os.SystemClock.sleep;

public class Clamp {
    static final double OPEN_CLAMP = 1;     // Clamp open position
    static final double CLOSE_CLAMP = 0;    // Clamp close position
    static final int    SLEEP_FOR_SERVO_MOTORS = 200;

    private Servo clamp = null;
    public HardwareMap hardwareMap;

    public  Clamp(HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

    public void Initialize ()
    {
        clamp = hardwareMap.get(Servo.class, "clamp");
        clamp.setPosition(CLOSE_CLAMP);

        //Wait for the clamp to go to the desired position
        sleep(1000);
    }

    public void OpenOrClose()
    {
        double clampStatus = clamp.getPosition();
        double setClampPosition = CLOSE_CLAMP;

        if(OPEN_CLAMP == clampStatus)
        {
            setClampPosition = CLOSE_CLAMP;
        }
        else if (CLOSE_CLAMP == clampStatus)
        {
            setClampPosition = OPEN_CLAMP;
        }
        else
        {
            return;
        }
        clamp.setPosition(setClampPosition);
        sleep(SLEEP_FOR_SERVO_MOTORS);
    }
}
