package org.firstinspires.ftc.teamcode;

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

/**
 * This program will go straight  hit the capball  and park the bot at the center vertex
 */

@Autonomous(name="XtremeV Red", group="Team10515")
@Disabled
public class Team10515AutoRed extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.4;
    static final double     BACKWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.1;


    boolean redColor = false;
    boolean blueColor = false;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.colorSensor.enableLed(false);
        //calibrateGyro();
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        String angle = formatAngle(angles.angleUnit, angles.firstAngle);

        telemetry.addData("heading",angle);
        telemetry.addData("firstAngle",angles.firstAngle);


        //sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       String glyphPosition = vuforiaCapture();
        telemetry.addData("The position is" ,glyphPosition);
        telemetry.update();
        //handDown();
        sleep(100);
        //clawClose();
        ElapsedTime time = new ElapsedTime();
        time.reset();

        robot.colorSensor.enableLed(true);
     //   while (opModeIsActive()) {
       while (time.time() < 10) {
            if (robot.colorSensor.red() > robot.colorSensor.blue() + 3) {
                redColor = true;
                telemetry.addData("color", "RED");
                telemetry.update();
                goStraight(FORWARD_SPEED,1.3);
               stopRobot();
                break;

            } else if (robot.colorSensor.blue() > robot.colorSensor.red() + 3) {
                blueColor = true;
                telemetry.addData("color", "BLUE");
                telemetry.update();
                goBack(BACKWARD_SPEED,0.8) ;
                stopRobot();
                break;
            } else {
                telemetry.addData("color", "UNKNOWN");
                telemetry.update();

            }
        }
        robot.colorSensor.enableLed(false);
        //sleep(1000);
      //  handUp();


        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angle = formatAngle(angles.angleUnit, angles.firstAngle);


        if (angles.firstAngle > 2.0  ) {

            turnRight(0.3,0.2);

            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }else  if (angles.firstAngle < -2.0  ) {
            turnLeft(0.3,0.2);

        }

            telemetry.addData("heading",angle);
        telemetry.addData("firstAngle",angles.firstAngle);
        telemetry.update();
        sleep(2000);

       // goStraight(FORWARD_SPEED,1.0);
        if (glyphPosition == "LEFT")
        {
            goBack(BACKWARD_SPEED,.1);
        }
        else if (glyphPosition == "CENTER")
        {
            hRight(FORWARD_SPEED, .5);
        }
        else if (glyphPosition == "RIGHT")
        {
            hLeft(FORWARD_SPEED,1.0);
        }
        else
        {
            telemetry.addData("Glyph","Random Placement into LEFT");
            telemetry.update();

        }
       //telemetry.addData("Heading is",robot.gyroSensor.getHeading());
        //telemetry.update();
        //sleep(5000);
        stopRobot();
        //Vuforia read image
/*
        goStraight(FORWARD_SPEED,.4);
        stopRobot();

         while (robot.gyroSensor.getHeading() < 358)
        {
            turnLeft(TURN_SPEED,1);
        }
        if (glyphPosition == ("LEFT"))
        {
            hLeft(INIT_FORWARD_SPEED,.2);
        }
        else if (glyphPosition.equals("RIGHT"))
        {
            hRight(INIT_FORWARD_SPEED,.2);
        }
        else if (glyphPosition.equals("CENTER"))
        {
            goBack(INIT_FORWARD_SPEED,.2);
        }
        else
        {
            stopRobot();
        }
        //goStraight(FORWARD_SPEED,1.0);
        //goBack(BACKWARD_SPEED,2.0);
       // turnRight(TURN_SPEED,0.15);
      //  goStraight(FORWARD_SPEED,.5);
        //stopRobot();
*/
    }

 /*   public void calibrateGyro() {
        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().clear();
        telemetry.addData("Gyro Calibrating. Do Not Move %s","!");
        telemetry.update();



        if (robot.gyroSensor instanceof  ModernRoboticsI2cGyro){
            telemetry.addData("modern robotics gyro","true");
            telemetry.update();
            sleep(3000);
        }else{
            telemetry.addData(" modern robotics gyro","false");
            telemetry.update();
            sleep(3000);
        }
        robot.gyroSensor.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && robot.gyroSensor.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        robot.gyroSensor.resetZAxisIntegrator();

        int rawX = robot.gyroSensor.rawX();
        int rawY = robot.gyroSensor.rawY();
        int rawZ = robot.gyroSensor.rawZ();
        int heading = robot.gyroSensor.getHeading();
        //int integratedZ = robot.gyroSensor.getIntegratedZValue();
        telemetry.clear();
        telemetry.addData("Values are","%s, %s, %s",rawX,rawY,heading);
        telemetry.update();

    }
*/
    public String vuforiaCapture() {
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeYAHIn/////AAAAGfVr1aFjUEHlh1uCvvWMJFtG8Y1D0YvNXpfCJTXkpgrNedm+jaqR+2trR9dGNzyeuHUMqo42P7DuJIp1IPDBDF5oepx6kw121V3vAc3sR5F43oix5brWapqdLcvFYcdFmWqg3AvIy436p1bkMhhJgcVEzXzIususTncxlVaHDDohnS9zN38qFcbFeKWH8cLG8lbt+2sNqoGJgOQ1/Oq6wEf3ceIS1x2BsguyUtkPLG0OQALkjbktRMdfLHe34ldDuCddP1ekNgkvwauoxOJqYKJKZX15h3VZfRtnp4mArn6Bxx8vWITXm690wfsdAio1LrRGm+NBovMapDxs9IKJuiH53nEoYrvat8IGG9IhMp67";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        int i = 0;
        RelicRecoveryVuMark vuMark = null;
        while (i < 3) {
            i++;
            telemetry.addData("i","%s", i);
            sleep(2000);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            else
            {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        return vuMark.toString();
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
