package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Team10515HWDemo
{

    //public DcMotor  RF = null;
    public DcMotor  RR = null;
    //public DcMotor  LF = null;
    public DcMotor  LR = null;

    public Servo    claw   = null;
    public Servo    tilt   = null;

    //static final String  LEFT_FRONT_MOTOR = "LF";
    static final String  LEFT_REAR_MOTOR = "LR";
    //static final String  RIGHT_FRONT_MOTOR = "RF";
    static final String  RIGHT_REAR_MOTOR = "RR";
    static final String  CLAW = "claw";
    static final String  TILT = "tilt";

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Team10515HWDemo(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //LF   = hwMap.dcMotor.get(LEFT_FRONT_MOTOR);
        LR   = hwMap.dcMotor.get(LEFT_REAR_MOTOR);
        //RF  = hwMap.dcMotor.get(RIGHT_FRONT_MOTOR);
        RR  = hwMap.dcMotor.get(RIGHT_REAR_MOTOR);

        claw  = hwMap.servo.get(CLAW);
        tilt  = hwMap.servo.get(TILT);

        //LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);
        //RF.setDirection((DcMotor.Direction.REVERSE));
        RR.setDirection((DcMotor.Direction.REVERSE));

        claw.setDirection(Servo.Direction.REVERSE);


        // Set all motors to zero power
        //RF.setPower(0);
        RR.setPower(0);
        //LF.setPower(0);
        LR.setPower(0);

        claw.setPosition(0);
        claw.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

