package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;


/* This program will go straight  hit the capball  and park the bot at the center vertex  */
@Autonomous(name="D-Bot Depot", group="XtremeV")
public class DBotAuto1 extends DBotBase
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private double headingResetValue;


    private static final String VUFORIA_KEY = "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        this.headingResetValue = this.getAbsoluteHeading();

        //String position = getGoldPosition();
        String position = "RIGHT";
        //unLatch(1.0,5.5);
        stopRobot();
        sleep(100);

        //robot.calibrateGyro(); dont want to do this because if it doesnt come down straight
        //turn to the right 45 degree call turn_to_heading(45);
        //turn to the left 45 degree call turn_to_heading(0); //its relative to previous turn
        //to move forward 10 inches call move_forward(10,0.3);
        //to move back 10 inches call move_forward(10, 0, -0.3);
        //to move sideways to the right by 4 inches call move_sideways(90, 0.3, 4);
        //to move sideways to left by 4 inches call move_sideways(180, 0.3, 4)

        if (position.equals("RIGHT"))
            removeRight();
        else if (position.equals("CENTER"))
            removeCenter();
        else if(position.equals("LEFT"))
            removeLeft();
        else
          removeRight();

        // Start the logging of measured acceleration
        stopRobot();

    }

    public void removeCenter()
    {
        move_sideways_by_range(90, 0.40, 10);
        turn_to_heading(45);
        move_forward(40, -0.3);
        //repositionBotAntiClock(45);
        //moveStraight(-200,0.2);
        //markerDrop();
        //wallfollow(70,45,-.5,1,true,true);
        //moveStraight(600,0.2);
        //moveSide(300,0.2);
        goToCrater();
        //sweep mineral in
    }
    public void removeLeft()
    {
        //moveSide(990,0.2, false);
        //turn_to_heading(45);
        //moveSide(990,.2, false);
        markerDrop();
        //wallfollow(70,45,-.5,1,true,true);

        //repositionBot(20.0);
        //sleep(2000);
//        moveStraight(-1450,0.2);
//        repositionBotAntiClock(45.0);
//        moveSide(800,0.2);
//        moveStraight(-100,0.2);
//        markerDrop();
//        moveStraight(800,0.2);
//        moveSide(300,0.2);
        goToCrater();
        //sweep mineral in and then get ready to go in to depot

    }
    public void removeRight()
    {
        move_sideways(90, 0.3, 2);
        sleep(100);
        move_forward(1,0.3);
        sleep(100);
        turn_to_heading(45);
        //move_sideways(90, 0.3, 40);
        sleep(100);
        move_sideways_by_range(90, 0.3, 4.5);
        sleep(100);
        move_forward(20,0.3);
        sleep(100);
        //markerDrop();
        //sleep(100);
        move_forward(70, -0.3);

        //markerDrop();
        //wallfollow(70,45,-.5,1,true,true);
        //goToCrater();
        //sweep mineral in and then get ready to go in to depot

    }
  private void goToCrater()
    {

        //moveStraight(1700,.2);
        //moveSide(230,.2);
        //moveStraight(800,.2);
        //repositionBotAntiClock(65);
        //parkCrater(0,.5);
    }


    public String getGoldPosition() {
        String pos = "Error";

        try {

            initVuforia();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                telemetry.update();
            }
            if (tfod != null) {
                sleep(200);
                tfod.activate();
                double getTime = getRuntime();
                while (opModeIsActive() && (pos == "Error" || getRuntime() - getTime <= 3)) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        pos = "LEFT";
                                        telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        pos = "RIGHT";
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        pos = "CENTER";
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        catch (Exception ex) {
            pos = "Error";
            telemetry.addData("Don't know Gold Mineral Position", "Error");
        }
        finally {
            if (tfod != null) {
                tfod.shutdown();
                vuforia = null;
            }
        }
        telemetry.update();
        return pos;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine. */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /* Initialize the Tensor Flow Object Detection engine. */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    private double getAbsoluteHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }
}
