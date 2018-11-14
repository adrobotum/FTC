package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autotest", group="test")
public class Autotest extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    @Override
    public void init() {


        telemetry.addData("Status", "Initialized");
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {
    }



}