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
    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private double headingResetValue;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //sleep(2000);

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        this.headingResetValue = this.getAbsoluteHeading();

       // String position = getGoldPositionHardCode();
        String position = getGoldPosition();
        sleep(500);
        latchUp(1.0,5.5);
        stopRobot();
        sleep(100);

        //turn to the right 45 degree call turn_to_heading(45);
        //turn to the left 45 degree call turn_to_heading(0); //its relative to previous turn
        //to move forward 10 inches call move_forward(10,0.3);
        //to move back 10 inches call move_forward(10, 0, -0.3);
        //to move sideways to the right by 4 inches call move_sideways(90, 0.3, 4);
        //to move sideways to left by 4 inches call move_sideways(180, 0.3, 4)


        if (position.equals("Right"))
            removeRight();
        else if (position.equals("Center"))
            removeCenter();
        else if(position.equals("Left"))
            removeLeft();
        else
          removeRight();

        // Start the logging of measured acceleration
        stopRobot();

    }

    public void removeCenter()
    {
        move_sideways_by_range(90, 0.40, 10);
        sleep(100);
        turn_to_heading(45);
        sleep(100);
        move_right(0.3,2.5);
        sleep(100);
        move_left(0.3,2.5);
        sleep(100);
        move_back(0.3,7);
        //markerDrop();
        goToCrater();

    }
    public void removeLeft()
    {
        move_back(0.3,2);
        sleep(100);
        move_right(0.3,12);
        sleep(100);
        turn_to_heading(135);
        sleep(100);
        move_forward(0.3,2);
        move_left(0.3,45);
        sleep(100);
        turn_to_heading(138);
        sleep(50);
        move_forward(0.3,35);
        sleep(100);
        //markerDrop();
        turn_to_heading(45);
        sleep(100);
        move_back(0.3,6);
        sleep(200);
        //markerDrop();
        goToCrater();

    }
    public void removeRight()
    {
        move_right(0.3, 2);
        sleep(100);
        //move_forward(1,0.3);
        move_forward(0.3,1);
        sleep(100);
        turn_to_heading(45);
        sleep(100);
        //move_sideways(90, 0.3, 40);
        move_forward(0.3,2);
        sleep(100);
        move_sideways_by_range(90, 0.3, 5);
        sleep(100);
        turn_to_heading(45);
        sleep(100);
        move_right(0.3,2);
        sleep(100);
        move_left(0.3,2);
        sleep(100);
        move_forward(0.3,35);
        sleep(100);

        move_back(0.3,4);
        goToCrater();

    }
  private void goToCrater()
    {
        move_sideways_by_range(90,0.3,3);
        markerDrop();
        sleep(100);
        move_back(0.3,53);
        sleep(100);
        move_left_by_range(0.3,4.75);
        sleep(100);
        turn_to_heading(225);
        sleep(100);
        move_forward(0.3,16);
        //move_sideways(-90,0.3,3);
        sleep(100);
        //move_sideways(0,0.3,30);

        robot.AA.setPower(.6);
        robot.AE.setPower(0.4);
        sleep(300);


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
