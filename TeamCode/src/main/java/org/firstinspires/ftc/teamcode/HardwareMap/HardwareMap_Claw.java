package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * WAGS:  Naming convention is camel case!
 *
 *          front
 *    (LF)--------(RF)
 *    |    robot   |
 *   (LB)--------(RB)
 *        back
 *
 * Motor channel:  Left Front (LF) drive motor:        "leftFront"
 * Motor channel:  Right Front (RF) drive motor:        "rightFront"
 * Motor channel:  Left Back (LB) drive motor:        "leftBack"
 * Motor channel:  Right Back (RB) drive motor:        "rightBack"
 */

public class HardwareMap_Claw
{
    /* Public OpMode members. */
    public Servo leftClaw  = null;
    public Servo rightClaw  = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap_Claw(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        leftClaw = hwMap.get(Servo.class, "claw");
        rightClaw = hwMap.get(Servo.class, "claw");
        //claw.setMode(Servo.RunMode.STOP_AND_RESET_ENCODER);
        leftClaw.setDirection(Servo.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        //claw.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}