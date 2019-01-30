package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Bot 3 Teleop", group = "XtremeV")
public class Bot3Telop extends OpMode
{
    /* Declare OpMode members. */
    public DcMotor  liftMotor = null;
    public Servo depositor = null;

    final double LIFT_UP = 1.00;
    final double LIFT_DOWN = -1.00;

    static final String   DEPOSITOR   = "D";
    static final String  LIFT_MOTOR   = "L";

    double  clawOffset  = 0.0 ;                  // Servo mid position
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Run code on INIT */
    @Override
    public void init() {
        liftMotor = hardwareMap.dcMotor.get(LIFT_MOTOR);
        depositor = hardwareMap.servo.get(DEPOSITOR);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Rover");    //
        updateTelemetry(telemetry);
    }

    /* Loop after INIT, NOT used. We can add telemetry for testing */
    @Override
    public void init_loop() {
    }

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
        telemetry.addData("Started", "Ready for Exploration");
        telemetry.update();
        runtime.reset();
    }

    /* Run this in a loop, process drive station input, till STOP */
    @Override
    public void loop() {

    if (gamepad1.y) {
        this.liftMotor.setPower(LIFT_UP);
    }
    else if (gamepad1.a) {
        this.liftMotor.setPower(LIFT_DOWN);
    }
    else
        this.liftMotor.setPower(0);

    if (gamepad1.b){
      if (runtime.time() > 20.0) {
            clawOffset = clawOffset + 0.02;
            clawOffset = Range.clip(clawOffset, 0, 0.9);
            this.depositor.setPosition(1 - clawOffset);
            runtime.reset();
         }
    } else if (gamepad1.x) {
        if (runtime.time() > 20.0) {
            clawOffset = clawOffset - 0.02;
            clawOffset = Range.clip(clawOffset, 0, 1);
            this.depositor.setPosition(1 - clawOffset);
            runtime.reset();
        }
    }
}
}