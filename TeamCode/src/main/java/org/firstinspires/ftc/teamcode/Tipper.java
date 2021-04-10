package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

public class Tipper {
    static final double TIPPER_POSITION_MAX = 1;    //Tipper max position
    static final double TIPPER_POSITION_MIN = 0;    //Tipper min position
    static final int    SLEEP_FOR_SERVO_MOTORS = 2000;

    private Servo tipper = null;
    public HardwareMap hardwareMap;

    public Tipper(HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

    public void Initialize ()
    {
        tipper = hardwareMap.get(Servo.class, "tipper");
        tipper.setPosition(TIPPER_POSITION_MIN);
    }

    public void PushOrPull(boolean userInput)
    {
        if(userInput)
        {
            double clampStatus = tipper.getPosition();
            tipper.scaleRange(0, 1);

            if(TIPPER_POSITION_MAX == clampStatus)
            {
                tipper.setPosition(TIPPER_POSITION_MIN);
            }
            else if (TIPPER_POSITION_MIN == clampStatus)
            {
                tipper.setPosition(TIPPER_POSITION_MAX);
            }
            sleep(SLEEP_FOR_SERVO_MOTORS);
        }
    }
}
