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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


public abstract class DBotBase extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    static final String VUFORIA_KEY = "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
    public double ticks_per_inch = 36.9;//37.4; //39.68; //35.4;      //wheel_encoder_ticks / (wheel_diameter * Math.PI);
    //private double wheel_encoder_ticks = ticks_per_inch * wheel_diameter * Math.PI;   //original 537.6

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    ElapsedTime gs_speed_timer = new ElapsedTime();
    boolean gs_first_run = true;
    int hard_stop = 5;  //seconds per operation i.e. move for this much or less

    public DBotBase() {
    }

    public void markerDrop() {
        robot.marker.setPosition(0);
        sleep(400);
        robot.marker.setPosition(0.2);
        //robot.marker.setPosition(0.8);

    }
    public void markerUp()
    {
        robot.marker.setPosition(0.5);
    }

    public void resetEncoders() {
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

        //resetEncoders();
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

    /*
    public String vuforiaCapture() {
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeYAHIn/////AAAAGfVr1aFjUEHlh1uCvvWMJFtG8Y1D0YvNXpfCJTXkpgrNedm+jaqR+2trR9dGNzyeuHUMqo42P7DuJIp1IPDBDF5oepx6kw121V3vAc3sR5F43oix5brWapqdLcvFYcdFmWqg3AvIy436p1bkMhhJgcVEzXzIususTncxlVaHDDohnS9zN38qFcbFeKWH8cLG8lbt+2sNqoGJgOQ1/Oq6wEf3ceIS1x2BsguyUtkPLG0OQALkjbktRMdfLHe34ldDuCddP1ekNgkvwauoxOJqYKJKZX15h3VZfRtnp4mArn6Bxx8vWITXm690wfsdAio1LrRGm+NBovMapDxs9IKJuiH53nEoYrvat8IGG9IhMp67";

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

        // For convenience, gather together all the trackable objects in one easily-iterable collection
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
    */

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
            new_speed = (ticks_traveled - gs_previous_ticks_traveled) / ticks_per_inch;
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
            wheel_power = (0.85 * degrees_to_turn) / 100;
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

    public void move_forward(double inches_to_travel, double speed, double unused) {
        double starting_speed = .05; //starting speed
        double current_speed = starting_speed;
        double speed_adjustment; //how much to increase or decrease by
        double adjustment_interval = 5;  //increase or decrease after this interval in millisec
        double optimal_braking = 10; //braking distance in inches

        int ticks_to_travel;
        double remaining_inches;

        boolean destination_reached = false;
        boolean going_backwards = false;

        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;

        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;

        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;

        //this is when we should be done
        ElapsedTime timeout_timer = new ElapsedTime();
        //this is when we adjust speed
        ElapsedTime adjustment_timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //check if robot needs to go forward (+ve speed) or backward (-ve speed passed)
        if (speed < 0) {
            going_backwards = true;                 //the bot needs to go backwards
            current_speed = -current_speed;        //starting speed will be -ve
            //speed_adjustment = -speed_adjustment;  //speed ramp up will be -ve as well
        }

        //calculate initial ticks to travel
        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        //reset encoders before beginning
        resetEncoders();

        //get initial encoder values for all 4 wheels
        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        timeout_timer.reset();
        adjustment_timer.reset();

        while (opModeIsActive() && !destination_reached && timeout_timer.seconds() < hard_stop) {
            telemetry.addData("Speed is", current_speed);
            telemetry.update();

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

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);
            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            //ramp up or ramp down only when adjustment interval is reached i.e. every 10 millisec
            //if (adjustment_timer.time() > adjustment_interval) {
            if (remaining_inches >= optimal_braking) {
                //adjust only if its not the speed passed to the method
                if (Math.abs(current_speed) < Math.abs(speed)) {
                    //increase the speed
                    speed_adjustment = 0.01; //remaining_inches / (inches_to_travel * 100);
                    if (going_backwards)
                        speed_adjustment = -speed_adjustment;

                    current_speed += speed_adjustment;

                    adjustment_timer.reset();
                } else {
                       /* telemetry.addData("current speed is",current_speed);
                        telemetry.addData("remaining inches is",remaining_inches);
                        telemetry.update();*/
                }
            } else { //start ramping down
                if (Math.abs(current_speed) > Math.abs(starting_speed)) {
                    //decrease the speed
                    speed_adjustment = 0.025; //remaining_inches / (inches_to_travel * 100);
                    if (going_backwards)
                        speed_adjustment = -speed_adjustment;

                    current_speed -= speed_adjustment;

                    adjustment_timer.reset();
                        /*telemetry.addData("current speed is",current_speed);
                        telemetry.addData("remaining inches is",remaining_inches);
                        telemetry.update();*/
                } else {
                        /*telemetry.addData("else current speed is",current_speed);
                        telemetry.addData("else remaining inches is",remaining_inches);
                        telemetry.update();*/
                }
            }
            //}
            /*
            //ramp up or ramp down only when adjustment interval is reached i.e. every 10 millisec
            if (adjustment_timer.time() > adjustment_interval) {
                //adjust only if its not the speed passed to the method
                if (remaining_inches >= optimal_braking) {
                    if (Math.abs(current_speed) < Math.abs(speed)) {
                        //increase the speed
                        current_speed += speed_adjustment;

                        adjustment_timer.reset();
                    }
                }
                else { //start ramping down
                    if (Math.abs(current_speed) > Math.abs(starting_speed * 2)) {
                        //decrease the speed
                        current_speed -= speed_adjustment;

                        adjustment_timer.reset();
                    }
                }
                */


        }

        stopRobot();
      /*  telemetry.addData("FL",robot.FL.getCurrentPosition());
        telemetry.addData("FR",robot.FR.getCurrentPosition());
        telemetry.addData("RL",robot.RL.getCurrentPosition());
        telemetry.addData("RR",robot.RR.getCurrentPosition());
        telemetry.update();
        */
        sleep(5000);

    } // end of go_forward

    public void move_forwardTime(double inches_to_travel, double speed) {
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

        while (opModeIsActive() && !destination_reached && timeout_timer.seconds() < 1) {

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

            //if (actual_speed > 0.2) {  // if we're going less than this we aren't moving.
            //    timeout_timer.reset();
            //}

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
        telemetry.addData("FL", robot.FL.getCurrentPosition());
        telemetry.addData("FR", robot.FR.getCurrentPosition());
        telemetry.addData("RL", robot.RL.getCurrentPosition());
        telemetry.addData("RR", robot.RR.getCurrentPosition());
        telemetry.update();
        sleep(5000);

    } // end of go_forward

    public double move_right(double speed, double inches) {
        return move_sideways(90, speed, inches);
    }

    public double move_left(double speed, double inches) {
        return move_sideways(-90, speed, inches);
    }

    public double move_forward(double speed, double inches) {
        return move_sideways(0, speed, inches);
    }

    public double move_back(double speed, double inches) {
        return move_sideways(180, speed, inches);
    }

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

    public double move_right_by_range(double speed, double inches_from_wall)
    {
        return move_sideways_by_range(90, speed, inches_from_wall);
    }

    public double move_left_by_range(double speed, double inches_from_wall)
    {
        return move_sideways_by_range(-90, speed, inches_from_wall);
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

        boolean moveLeftFlag = false;
        if (get_right_distance() <= inches_from_wall) {
            moveLeftFlag = true;
        }

        while (opModeIsActive() && !destinationreached) {
            //convert inches from wall to inches
            if (moveLeftFlag) {
                inches = inches_from_wall - get_right_distance();
            } else
                inches = get_right_distance() - inches_from_wall;


            if (!moveLeftFlag && inches <= inches_from_wall) {
                destinationreached = true;
            } else if (moveLeftFlag && inches <= 0) {
                destinationreached = true;
            }

            //if going back then
            //right distance = get right distance
            //if previous right > right distance
            //turn right by 1 degree
            //if previous right < right distance
            //turn left

            //if going forward then
            //right distance = get right distance
            //if previous right > right distance
            //turn left by 1 degree
            //if previous right < right distance
            //turn right



            telemetry.addData("Inches to travel", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

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

    public double move_forward_by_range(double speed, double inches_from_wall)
    {
        return move_by_range(0, speed, inches_from_wall);
    }

    public double move_backward_by_range(double speed, double inches_from_wall)
    {
        return move_by_range(180, speed, inches_from_wall);
    }

    public double move_by_range(double heading, double speed, double inches_from_wall) {
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

        boolean moveBackFlag = false;
        if (get_Front_Distance() <= inches_from_wall) {
            moveBackFlag = true;
        }

        while (opModeIsActive() && !destinationreached) {
            //convert inches from wall to inches
            if (moveBackFlag) {
                inches = inches_from_wall - get_Front_Distance();
            } else
                inches = get_Front_Distance() - inches_from_wall;


            if (!moveBackFlag && inches <= inches_from_wall) {
                destinationreached = true;
            } else if (moveBackFlag && inches <= 0) {
                destinationreached = true;
            }


            telemetry.addData("Inches to travel", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

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

    public double get_right_distance() {
        return robot.rightSensor.getDistance(DistanceUnit.INCH);
    }

    public double get_Front_Distance(){
        return robot.frontSensor.getDistance(DistanceUnit.INCH);
    }


    public String getGoldPositionHardCode(){
        return "Right";
    }
    public String getGoldPosition() {
        String pos = "Error";

        try {

//            initVuforia();
//            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//                initTfod();
//            } else {
//                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//                telemetry.update();
//            }

//            if (tfod != null) {
//                tfod.activate();
//            }

            ElapsedTime goldTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            goldTimer.reset();

            while (opModeIsActive() && pos == "Error" && goldTimer.time() < 3000) {
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
                                    pos = "Left";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    pos = "Right";
                                } else {
                                    pos = "Center";
                                }
                            }
                        }
                        telemetry.update();
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
        telemetry.addData("Gold Mineral Position", pos);
        telemetry.update();
        return pos;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        if (tfod != null) {
            tfod.activate();
        }

    }

}
