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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="homestart", group="Linear Opmode")
public class homestart extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    // DcMotor updown  = hardwareMap.get(DcMotor.class, "updown");
    private DcMotor motor4 = null;
    private DcMotor crane = null;
    private DcMotor arm = null;
    //boolean hasyellowbeenfound = false;


    OpenGLMatrix lastLocation = null;
VuforiaLocalizer vuforia;
     @Override
    public void runOpMode() {

         telemetry.addData("Status", "Initialized");
         telemetry.update();


         motor1 = hardwareMap.get(DcMotor.class, "motor1");
         motor2 = hardwareMap.get(DcMotor.class, "motor2");
         arm = hardwareMap.get(DcMotor.class, "arm");
         motor3 = hardwareMap.get(DcMotor.class, "motor3");
         motor4 = hardwareMap.get(DcMotor.class, "motor4");
         crane = hardwareMap.get(DcMotor.class, "crane");


         arm.setDirection(DcMotor.Direction.FORWARD);
         motor1.setDirection(DcMotor.Direction.FORWARD);
         motor2.setDirection(DcMotor.Direction.FORWARD);
         motor3.setDirection(DcMotor.Direction.REVERSE);
         motor4.setDirection(DcMotor.Direction.REVERSE);
         crane.setDirection(DcMotor.Direction.FORWARD);

         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();
        /*
        LOWER ROBOT
        */


         // lowerrobot();
         crane.setPower(-0.5);
         sleep(3000);
         crane.setPower(0);

         drive("left", 500);
         sleep(500);

         drive("back", 2000);
         sleep(2000);
         turn("L", 45);
         drive("forward", 2000);
         sleep(2000);
     }

public void turn(String direction, long ms){
        if(direction == "L"){
            motor1.setPower(-1);
            motor2.setPower(-1);
            motor3.setPower(1);
            motor4.setPower(1);
            sleep(ms);
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }else if(direction == "R"){

            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(-1);
            motor4.setPower(-1);
            sleep(ms);
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }


}

public void stoprobot(){
    motor1.setPower(0);
    motor2.setPower(0);
    motor3.setPower(0);
    motor4.setPower(0);
}
    public void drive(String loc,
                             long ms) {

        if(loc == "forward") {
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(1);
            motor4.setPower(1);
            sleep(ms);
         stoprobot();
        }
        if(loc == "back") {
            motor1.setPower(-1);
            motor2.setPower(-1);
            motor3.setPower(-1);
            motor4.setPower(-1);
            sleep(ms);
            stoprobot();
        }
        if(loc == "left") {
            motor1.setPower(-1);
            motor2.setPower(1);
            motor3.setPower(-1);
            motor4.setPower(1);
            sleep(ms);
            stoprobot();
        }
        if(loc == "right") {
            motor1.setPower(1);
            motor2.setPower(-1);
            motor3.setPower(1);
            motor4.setPower(-1);
            sleep(ms);
            stoprobot();
        }



    }

}


