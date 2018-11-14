package org.firstinspires.ftc.teamcode;



import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@TeleOp(name="ftctest", group="Iterative Opmode")
public class ftctest extends OpMode
{

AndroidTextToSpeech tts = new AndroidTextToSpeech();



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        telemetry.addData("I am", "Robot vxi");
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        boolean btn = gamepad1.left_stick_button;
        double plus = drive + turn;
        double min = drive - turn;
        telemetry.addData("Status", "drive-turn " + min);
        telemetry.addData("Status", "drive+turn " + plus);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
  //left_trigger

        if(gamepad1.x){
            tts.initialize();
            tts.setLanguage("nld");
tts.speak("Hallo, ik ben een robot.");
        }
        if(gamepad1.y){
            arm.setPower(0.1);
        }
        if(gamepad1.a){
            arm.setPower(-0.1);
        }

        if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
            leftPower = -gamepad1.left_trigger;
            rightPower = gamepad1.right_trigger;
        }else{
if(gamepad1.left_bumper){
    leftPower = 0.5;
}else{
    leftPower = 0;
}
            if(gamepad1.right_bumper){
                rightPower = -0.5;
            }else{
                rightPower = 0;
            }
        }
       // leftPower  = gamepad1.left_stick_y ;
       // rightPower = -gamepad1.right_stick_y ;
      /*  if(gamepad1.left_bumper) {
            if (gamepad1.left_stick_y > 0) {
                leftPower = 1;

            } else {

                leftPower = -1;
            }
        }
        if(gamepad1.right_bumper) {
            if(gamepad1.right_stick_y > 0){
                rightPower = -1;

            }else{

                rightPower= -0;
            }
            //leftPower  = gamepad1.left_stick_y ;
           // rightPower = -gamepad1.right_stick_y


        }
*/
        // Send calculated power to wheels

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}