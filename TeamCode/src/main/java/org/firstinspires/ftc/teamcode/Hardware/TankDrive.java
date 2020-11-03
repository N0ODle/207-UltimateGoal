package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;

public class TankDrive {

    static public Bot bot;


    public TankDrive(Bot bot) {
        this.bot = bot;
    }


    public static void driveBot (double leftStickY, double rightStickX, double rightTrigger, double leftTrigger, double rightStickY, double leftStickX){
        if(leftStickY > 0.15 || leftStickY < -0.15){
            bot.BL.setPower(-leftStickY);
            bot.FL.setPower(leftStickY);
        }
        else{
            bot.BL.setPower(0.0);
            bot.FL.setPower(0.0);
        }
        if(rightStickY > 0.15 || rightStickY < -0.15){
            bot.FR.setPower(rightStickY);
            bot.BR.setPower(-rightStickY);
        }
        else {
            bot.BR.setPower(0.0);
            bot.FR.setPower(0.0);
        }

    }
}