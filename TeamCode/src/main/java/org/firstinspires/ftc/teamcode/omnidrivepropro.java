/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="omnisrivepropro", group="Linear Opmode")
public class omnidrivepropro extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   // private DcMotor leftDrive = null;
   // private DcMotor rightDrive = null;
   private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
double armpos = 0;
double grabpos = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
       // leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
       // rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        DcMotor arm  = hardwareMap.get(DcMotor.class, "arm");

        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
      /*  Servo arm = hardwareMap.get(Servo.class, "arm");
        Servo g1 = hardwareMap.get(Servo.class, "g1");
        Servo g2 = hardwareMap.get(Servo.class, "g2");*/
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;
            //left stick: omni drive x and y
            double leftPower_y = -gamepad1.left_stick_y ;
            double rightPower_y = -gamepad1.left_stick_y ;
            double leftPower_x = gamepad1.left_stick_x ;
            double rightPower_x = gamepad1.left_stick_x ;

            //right stick: turning around axis

            double turn = gamepad1.right_stick_x ;

            //power to variable:
            // - en + misschien aanpassen want niet x-as positief of negatief links en rechts?

            double left1Powerxy = Range.clip(leftPower_y + leftPower_x, -1.0, 1.0);
            double left2Powerxy = Range.clip(leftPower_y - leftPower_x, -1.0, 1.0);
            double right1Powerxy = Range.clip(rightPower_y + rightPower_x, -1.0, 1.0);
            double right2Powerxy = Range.clip(rightPower_y - rightPower_x, -1.0, 1.0);




            double left1Power = Range.clip(left1Powerxy - turn, -1.0, 1.0);
            double left2Power = Range.clip(left2Powerxy - turn, -1.0, 1.0);
            double right1Power = Range.clip(right1Powerxy + turn, -1.0, 1.0);
            double right2Power = Range.clip(right2Powerxy + turn, -1.0, 1.0) ;

            //set power to motor:


            //absolute direction: left:
            if (gamepad1.x)
            {
                motor1.setPower(-right2Power);
                motor2.setPower(left1Power);
                motor3.setPower(-left2Power);
                motor4.setPower(right1Power);
            }
            //absolute direction: down:
            else if (gamepad1.a)
            {
                motor1.setPower(-right1Power);
                motor2.setPower(-right2Power);
                motor3.setPower(-left1Power);
                motor4.setPower(-left2Power);
            }
            //absolute direction: right:
            else if (gamepad1.b)
            {
                motor1.setPower(left2Power);
                motor2.setPower(-right1Power);
                motor3.setPower(right2Power);
                motor4.setPower(-left1Power);
            }
            else
            {
                //forward/normal drive direction
                motor1.setPower(left1Power);
                motor2.setPower(left2Power);
                motor3.setPower(right1Power);
                motor4.setPower(right2Power);
            }
            // arm control

            if( gamepad1.dpad_up){
arm.setPower(0.6);
                //up
}else if(gamepad1.dpad_down){
                //down
arm.setPower(-0.6);
}
arm.setPower(0);

/*

if(gamepad1.dpad_right){
grabpos -= 1;
                //open
}else if(gamepad1.dpad_left){
grabpos += 1;
                //close
}
//apply arm positions
            arm.setPosition(armpos);
            g1.setPosition(grabpos);
            g2.setPosition(grabpos);
*/



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
           // double drive = -gamepad1.left_stick_y;
           // double turn  =  gamepad1.right_stick_x;
           // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //leftDrive.setPower(leftPower);
            //rightDrive.setPower(rightPower);
            telemetry.addData("motor1 Position", "" +  motor1.getCurrentPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            idle();
        }
    }
}
