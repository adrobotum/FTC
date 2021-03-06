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
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="auto", group="Linear Opmode")
public class Auto extends LinearOpMode {

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

         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


         parameters.vuforiaLicenseKey = "AWVeyNv/////AAABmVE9v4SQAEhohP8/ReQFrZY5CCjekal9Xlt/c2w7UtFRaxT00XE2rIMuZxLLMj+N0LRS0d9Znhv1hkTTsx1Cjp5/lDPau3yjvNeMTPdwKImabdbQypKO0AbS5VMSqWXbEIS2ZTu7dqVvO+KozvKsa4pHwmAv3VphDwMs6QN811tP20dn2xVDcOL3tzavpuJAfTwXb8wHhF3b8BMm/6zowmrjfyl6Wr9RPnth2qwSIORbTv9SGq0u47WxWO6WfXyO6Y6LDknYcdEwVX6eynz4F/6cPVn7figk3gGr+epaxL0OfXLsADg+ZSqX4P3XoEAQyyCWddygnMZq3MtlNcVPIuXnZxhiXWqgTDS2isnf6NFs";
         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
         vuforia = ClassFactory.getInstance().createVuforia(parameters);
         VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
         VuforiaTrackable relicTemplate = relicTrackables.get(0);
         relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        telemetry.addData("Status", "Initialized");
        telemetry.update();


        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        crane = hardwareMap.get(DcMotor.class, "crane");


        //pullcontroller = hardwareMap.get(DcMotor.class, "pc");
       /* NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
      /*  Servo arm = hardwareMap.get(Servo.class, "arm");
        Servo g1 = hardwareMap.get(Servo.class, "g1");
        Servo g2 = hardwareMap.get(Servo.class, "g2");
*/

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

drive("back", 1000);
sleep(1000);



//while(true){
//    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//    telemetry.addData("VuMark", "%s visible", vuMark);
//    telemetry.update();
//}


                /*
        claim minerals
        */
        //NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /*long driven = 0;
        double red = colors.red;
        double green = colors.green;
double blue = colors.blue;
        if(gamepad1.a) {
            hasyellowbeenfound = true;
        }

      while (red > 0.02
              && blue < 0.1 && !hasyellowbeenfound){
            //drive further
            drive("left", 100);
            sleep(100);
            driven += 100;
            NormalizedRGBA colornew = colorSensor.getNormalizedColors();
            red = colornew.red;
            green = colornew.green;
            blue = colornew.blue;
      }


        long foundat = driven;
        hasyellowbeenfound = true;


drive("forward", 300);
        //sleep(200);
       // drive("back", 400);
drive("left", foundat);
drive("forward", 200);

//in vak
        arm.setPower(-0.5);
        sleep(500);
        arm.setPower(0);
        arm.setPower(0.5);
        sleep(500);
        arm.setPower(0);



/*sleep(200);
        drive("right", 350);
        drive("left", 1500);
turn("left", 250);

drive("left", 1500);*/

        /*
        go to crater
        */
//drive("forward", 1);
//sleep(1500);
        //       drive("back", 1);
        //      sleep(1000);
        // turn("L", 2);
        // sleep(2000);
        //  turn("R", 250);
        //  turn("L", 250);



/*
public void lowerrobot(){
updown.setPower(1);
sleep(2000);
updown.setPower(0);


}
*/
/*public void turn(String direction, long ms){
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


}*/
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


