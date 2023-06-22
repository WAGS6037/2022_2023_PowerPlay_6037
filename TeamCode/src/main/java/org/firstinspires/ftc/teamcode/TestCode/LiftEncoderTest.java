package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;

// name this opMode and determine a group
@TeleOp(name="LiftEncoderTest", group="Test?")
public class LiftEncoderTest extends OpMode{

    /* Declare OpMode members. */
    HardwareMap_CompetitionBot robot = new HardwareMap_CompetitionBot();

    @Override
    public void init() {

        /* Initialize the har5dware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello WAGS Driver!!");    //

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    @Override
    public void loop() {

        boolean isButtonLB2 = gamepad2.left_bumper;
        boolean isButtonRB2 = gamepad2.right_bumper;

        //double speed = 0.5;

        if (isButtonRB2) {
            robot.slideSystem.setPower(1);
            telemetry.addData("Button","RB");
            //A is retract
        } else if (isButtonLB2) {
            robot.slideSystem.setPower(-1);
            telemetry.addData("Button","LB");
            //A is retract
        }else {
            telemetry.addData("Button","None");
            robot.slideSystem.setPower(0);
        }
        telemetry.addData("Lift Position", String.format("%7d", robot.slideSystem.getCurrentPosition()));
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Good Job Team! We have STOPPED!!");

    }


}
