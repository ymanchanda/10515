package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


public abstract class DBotBase extends LinearOpMode {

    /* Declare OpMode members. */
    DBotAutoHWMap robot = new DBotAutoHWMap();
    ElapsedTime runtime = new ElapsedTime();
    //Orientation angles;
    public Orientation orientation;
    //Create the angle tracker
    public double angle = 0;
    private double current_heading;

    //amount of clicks per cm
    //public final double ENCDISTANCE = 34.5781466113;

    private double wheel_diameter = 3.937;      //size of wheels original 3.75
    public double ticks_per_inch = 35.4;      //wheel_encoder_ticks / (wheel_diameter * Math.PI);
    //private double wheel_encoder_ticks = ticks_per_inch * wheel_diameter * Math.PI;   //original 537.6

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    ElapsedTime gs_speed_timer = new ElapsedTime();
    boolean gs_first_run = true;
    int    hard_stop = 5;  //seconds per operation i.e. move for this much or less

    public DBotBase(){
    }

    VuforiaLocalizer vuforia;

    public void markerDrop()
    {
        robot.marker.setPosition(0);
        sleep(500);
        robot.marker.setPosition(0.8);

    }

    public void resetEncoders()
    {
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(100);

        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopRobot() {
        robot.RL.setPower(0.0);
        robot.RR.setPower(0.0);
        robot.FR.setPower(0.0);
        robot.FL.setPower(0.0);
        robot.LL.setPower(0.0);

        resetEncoders();
    }

    public void latchUp(double speed, double time) {
        robot.LL.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //  telemetry.update();
        }
    }

    public void latchDown(double speed, double time) {
        robot.LL.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

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
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        targetsRoverRuckus.activate();

        boolean targetVisible = false;
        int i = 0;

        while (!targetVisible && i < 5) {
            sleep(1000);
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    telemetry.update();
                    targetVisible = true;
                    return trackable.getName();
                }
            }
            i++;
        }

        if (!targetVisible)
            telemetry.addData("visible target", "not visible");

        telemetry.update();
        return "not visible";
    }

    public void initialize() {
        robot.init(hardwareMap);
        // calibrateGyro();
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);

        //robot.claw.setPosition(.5);
        // robot.liftMotor.setPower(ARM_UP_POWER);

        sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    private double getSpeed(double ticks_traveled) {
        double new_speed;

        if (gs_first_run) {
            gs_previous_ticks_traveled = ticks_traveled;
            gs_speed_timer.reset();
            gs_previous_speed = 1;
            gs_first_run = false;
        }

        if (gs_speed_timer.seconds() >= .1) {
            new_speed = (ticks_traveled - gs_previous_ticks_traveled) / 46.5;  // At max speed we travel about 4800 ticks in a second so this give a range of 0 - 10 for speed
            gs_speed_timer.reset();
            gs_previous_speed = new_speed;
            gs_previous_ticks_traveled = ticks_traveled;
        } else {
            new_speed = gs_previous_speed;
        }

        return new_speed;
    }

    private double shift_heading(double heading) {
        double shiftvalue = 3;
        heading = heading + shiftvalue;

        if (heading >= 360) {
            heading = heading - 360;
        } else if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }

    private double get_current_heading() {
        orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = orientation.firstAngle;

        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }
        current_heading = shift_heading(current_heading);
        return current_heading;
    }

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prev_heading = 0;
        ElapsedTime timeout_timer = new ElapsedTime();

        current_heading = get_current_heading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;
        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeout_timer.reset();
        prev_heading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeout_timer.seconds() < 2) {
            //wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;
            wheel_power = (0.75 * degrees_to_turn) / 100;
            if (go_right) {
                wheel_power = -wheel_power;
            }

            robot.FR.setPower(wheel_power);
            robot.RR.setPower(wheel_power);
            robot.FL.setPower(-wheel_power);
            robot.RL.setPower(-wheel_power);

            current_heading = get_current_heading();
            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prev_heading) > 1) {
                timeout_timer.reset();
                prev_heading = current_heading;
            }
        }
        stopRobot();
    } // end of turn_to_heading

    public void move_forward(double inches_to_travel, double speed) {
        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        double speed_increase = .05;
        double actual_speed;

        int ticks_to_travel;
        double remaining_inches;

        boolean destination_reached = false;
        boolean going_backwards = false;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int previous_ticks_traveled_L = 0;

        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;

        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;

        double previous_log_timer = 0;

        ElapsedTime timeout_timer = new ElapsedTime();

        if (speed < 0) {
            going_backwards = true;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
        }

        //inches_to_travel = inches_to_travel - lagreduction;
        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        log_timer.reset();
        timeout_timer.reset();

        while (opModeIsActive() && !destination_reached && timeout_timer.seconds() < hard_stop) {

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            robot.FR.setPower(current_speed);
            robot.RR.setPower(current_speed);
            robot.FL.setPower(current_speed);
            robot.RL.setPower(current_speed);

            ticks_traveled_FL = Math.abs(robot.FL.getCurrentPosition() - start_position_FL);
            ticks_traveled_RL = Math.abs(robot.RL.getCurrentPosition() - start_position_RL);
            ticks_traveled_FR = Math.abs(robot.FR.getCurrentPosition() - start_position_FR);
            ticks_traveled_RR = Math.abs(robot.RR.getCurrentPosition() - start_position_RR);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_FL, ticks_traveled_RL);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_FR, ticks_traveled_RR);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.2) {  // if we're going less than this we aren't moving.
                timeout_timer.reset();
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
            }

            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);
            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
            }
        }
        stopRobot();
        sleep(100);

    } // end of go_forward

    public double move_sideways(double heading, double speed, double inches) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;

        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !destinationreached) {

            ticks_traveled_FL = Math.abs(robot.FL.getCurrentPosition() - start_position_FL);
            ticks_traveled_RL = Math.abs(robot.RL.getCurrentPosition() - start_position_RL);
            ticks_traveled_FR = Math.abs(robot.FR.getCurrentPosition() - start_position_FR);
            ticks_traveled_RR = Math.abs(robot.RR.getCurrentPosition() - start_position_RR);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_FL, ticks_traveled_RL);
            highest_ticks_traveled_r = Math.max(ticks_traveled_FR, ticks_traveled_RR);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);
        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }

    public double move_sideways_by_range(double heading, double speed, double inches_from_wall) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !destinationreached) {
            //convert inches from wall to inches
            inches = get_right_distance() - inches_from_wall;

            telemetry.addData("Inches to travel", inches);
            telemetry.update();

            if (inches <= inches_from_wall) {
                destinationreached = true;
            }

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);
        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }

    private double go_straight_adjustment(double target_heading) {

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel
        double gs_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        current_heading = get_current_heading();

        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < .3) {
            gs_adjustment = 0;
        } else {
            gs_adjustment = (Math.pow((degrees_off + 2) / 5, 2) + 2) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        return gs_adjustment;

    } // end of go_straight_adjustment

    private double get_right_distance()
    {
        return robot.rightSensor.getDistance(DistanceUnit.INCH);
    }
}
