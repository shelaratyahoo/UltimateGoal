
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import static android.os.SystemClock.sleep;

public class RingDetection {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "4";
    private static final String LABEL_SECOND_ELEMENT = "1";
    private static final float MINIMUM_CONFIDENCE_LEVEL = 0.9f;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime;
    private static final String VUFORIA_KEY =
            "Acosqqn/////AAABmfIYQdPwIU7AoMKFurxSuUAu9hSR2hHarwzeIX67gzKEQf1vVpRSqa9rX6aTS4K0oaP/rr6rMgwOGhwV3vCQw7p2ZwhC8t1AZ02bM3eEBE/KIqkuqUe5deszpscm2hCQETXUitQVtbfkmabu9fqee/SqV4zCz1KrY5KiLfZr+rrM2Cs3jsRN0dvni2gpk2G/P9YtE6ZW2hSBTVDQ2EICQKJJeE9CJGg7woMujtlQLGVw/H13PMGBXDpa6R632rk6E7CT3lxosZhO9Td7ISXIfzpOrXQE0HvyZUqsfiCuFm4KPcGq/ickixEKD4S0OiHaRm6te8xaZPn8myjbAsKx16GEDsT5eSs+ZMlC+NHCGCm3";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public RingDetection(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime){
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
        Init();
    }

    public void Init(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();
        initTfod();
    }

    public void activateTF()
    {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void shutdownTF(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public int detectRing() {
        int count = 0;
        int numberOfRings = 0;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            boolean continueRingDetection = true;
            while(continueRingDetection) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format(" Accuracy=%d", i++), "%.03f", recognition.getConfidence());
                        if(recognition.getConfidence() > MINIMUM_CONFIDENCE_LEVEL){
                            continueRingDetection = false;
                            if(recognition.getLabel() != ""){
                                numberOfRings=Integer.parseInt(recognition.getLabel());
                            } else {
                                numberOfRings = 0;
                            }
                        }
                    }
                    telemetry.update();
                }
                else {
                    count++;
                    sleep(200);
                    if(count > 9){
                        numberOfRings = 0;
                        continueRingDetection = false;
                    }
                }
            }
        }
        return numberOfRings;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "ringWebCam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = MINIMUM_CONFIDENCE_LEVEL;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
