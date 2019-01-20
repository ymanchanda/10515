package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/* This program will go straight  hit the capball  and park the bot at the center vertex  */
@Autonomous(name="D-Bot Forward Range", group="XtremeV")
public class DBotTestForwardRange extends DBotBase {
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

        move_forward_by_range(0.3, 5);
        sleep(1000);
        move_backward_by_range(0.3, 10);
    }
}
