package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name="RRBlue", group="Team10515")
public class BlueProgram extends RR10515Base
{
    static final double INIT_FORWARD_SPEED = 0.1;
    static final double FORWARD_SPEED = 0.5;
    static final double BACKWARD_SPEED = 0.4;
    static final double TURN_SPEED = 0.1;

    private static final double CIRCUMFERENCE = 12.35;
    private static final double SIDEWAYS = 15.25;
    private GoldAlignDetector detector;

    boolean redColor = false;
    boolean blueColor = false;

    public static final String TAG = "Vuforia VuMark Sample";

    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initialize();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        latchUp(.5,.2);
        stopRobot();
        String teamColor = colorSenseRev();
        if (teamColor.equals("BLUE"))
        {
            blueColor = true;
            moveStraightEncoder(2240,FORWARD_SPEED);
            stopRobot();

        }
        if(teamColor.equals("RED"))
        {
            redColor = true;
            moveStraightEncoder(2240,FORWARD_SPEED);
            stopRobot();
        }
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        if (detector.getAligned())
        {
            detector.disable();
            moveStraightEncoder(1220,FORWARD_SPEED);
        }
        else
        {
          while(!detector.getAligned())
            {
                moveRSideEncoder(200,.5);
            }
        }

        //call vuforia and capture image
        //create if statment comparing vuforia image and color to understand where we are
        //move correct jewl

    }
}



