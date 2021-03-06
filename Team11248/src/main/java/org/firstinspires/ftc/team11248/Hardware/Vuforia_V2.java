package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.team11248.Old_Files.Robot11248;

/**
 * Created by tonytesoriero on 9/11/17.
 */

public class Vuforia_V2 {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public static final String vuforiaKey = "AeTwV0H/////AAAAGfe7ayWmjE9+nI9k65aoO+NQIIujZBIX8AxeoVDf9bwLLNvQ6QwvM+Clc3CE/8Pumv5guDuXMxkERpyJTzSb50PcrH9y/lJC9Zfh0FlPVkkvDnZVNsPEIEsg0Ta5oDlz1jIZmSB/Oxu2qRAyo4jXIsWSmDMdQdpNrwkyKbLfl/CT7PWe23RAdF8oQf5XqnSbKoapQali8MH4+HPOR8r13/k+cZv9eKqUvknmxZPiyJbp4oFzqrWDJSUqwTGQLEdbp76Hjrkuxu3Pa/I4jQSt3RRRbAUrZeV1Z79cLKg+22SvrhUKKzwxeEMcgp4rQzrMXhTL+wE+6sBczuguHmPtWA5w/NsUlevRaLbEionbyXYN";

    private ClosableVuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private RelicRecoveryVuMark lastVuMark = RelicRecoveryVuMark.UNKNOWN;
    private RelicRecoveryVuMark currentVuMark;
    boolean isVisible;


    OpenGLMatrix pose;

    private double[] lastPosition = {0, 0, 0};
    private double[] lastRotation = {0, 0, 0};

    public Vuforia_V2(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


    /*
    Init
     */

    public void init(){
        init(true, false);
    }

    public void init(boolean debug){
        init (true, debug);
    }

    public void init(boolean frontCamera, boolean debug){

        VuforiaLocalizer.Parameters parameters;

        if(debug) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        }else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = Vuforia_V2.vuforiaKey;

        parameters.cameraDirection = frontCamera?VuforiaLocalizer.CameraDirection.FRONT:VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        this.relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }


    /*
    Vumark Recognition
     */

    public void activateTracking(){
        this.relicTrackables.activate();
    }

    public void deactivateTracking(){
        this.relicTrackables.deactivate();
        vuforia.close();
    }

    public RelicRecoveryVuMark getLastImage(){
        update();
        return lastVuMark;
    }

   public RelicRecoveryVuMark getCurrentImage(){
       update();
       return currentVuMark;

   }

   public boolean isVisible(){
       update();
       return isVisible;
   }

   public void update(){
       currentVuMark = RelicRecoveryVuMark.from(this.relicTemplate);
       if(currentVuMark != RelicRecoveryVuMark.UNKNOWN) this.lastVuMark = currentVuMark;

       isVisible = (currentVuMark == lastVuMark && currentVuMark != RelicRecoveryVuMark.UNKNOWN);
   }


    /*
    Positional Information
     */

    // Millimeters
    public double[] getPosition(){

        pose = ((VuforiaTrackableDefaultListener)this.relicTemplate.getListener()).getPose();

        if(pose != null) {
            VectorF trans = pose.getTranslation();

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            lastPosition[0] = Math.round(trans.get(0));
            lastPosition[1] = Math.round(trans.get(1));
            lastPosition[2] = Math.round(trans.get(2));
        }

        return lastPosition;
    }

    // Degrees
    public double[] getRotation(){

        pose = ((VuforiaTrackableDefaultListener)this.relicTemplate.getListener()).getPose();

        if(pose != null) {
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the rotational components of the target relative to the robot
            lastRotation[0] = Math.round(rot.firstAngle); // Vertical Rotation
            lastRotation[1] = Math.round(rot.secondAngle); // Horizontal Rotation
            lastRotation[2] = Math.round(rot.thirdAngle); // Face rotation
        }

        return lastRotation;

    }

    public void resetCache(){
        lastVuMark = RelicRecoveryVuMark.UNKNOWN;
        currentVuMark = RelicRecoveryVuMark.UNKNOWN;
        isVisible = false;

        lastPosition[0] = 0;
        lastPosition[1] = 0;
        lastPosition[2] = 0;

        lastRotation[0] = 0;
        lastRotation[1] = 0;
        lastRotation[2] = 0;
    }


    /*
    Telemetry
     */

    public void printPositionTelemetry(){

    }

    public void printRotationTelemetry(){

    }

    public void printVuMarkTelemetry(){

    }

    public void printAutoTelemetry() {

        telemetry.addData("VuForia", "Last VuMark: " + getLastImage().toString());
        telemetry.addData("VuForia","Current VuMark: " + getCurrentImage().toString());
        telemetry.addData("VuForia","Visible: " + isVisible());
    }

    public void printTelemetry(){
        printAutoTelemetry();
        telemetry.addData("VuForia","Position: { " + getPosition()[0] + ", " + getPosition()[1] + ", " + getPosition()[2] +" }");
        telemetry.addData("VuForia","Rotation: { " + getRotation()[0] + ", " + getRotation()[1] + ", " + getRotation()[2] +" }" );
    }



}
