/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "color sensor".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "MR Color Test", group = "Sensor")
public class MRColorTest extends LinearOpMode {

    ColorSensor colorSensor1;    // Hardware Device Object
    ColorSensor colorSensor2;
    ColorSensor colorSensor3;


    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        float hsvValues2[] = {0F, 0F, 0F};
        float hsvValues3[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        final float values2[] = hsvValues2;
        final float values3[] = hsvValues3;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        boolean aPrevState = false;
        boolean aCurrState = false;
        boolean xPrevState = false;
        boolean xCurrState = false;


        // bLedOn represents the state of the LED.
        boolean bLedOn = true;
        boolean aLedOn = true;
        boolean xLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor1 = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensor2 = hardwareMap.colorSensor.get("colorSensor2");
        colorSensor3 = hardwareMap.colorSensor.get("colorSensorRight");

        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x1c));
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x2c));
        colorSensor3.setI2cAddress(I2cAddr.create8bit(0x3c));

        // Set the LED in the beginning
        colorSensor1.enableLed(bLedOn);
        colorSensor2.enableLed(aLedOn);
        colorSensor3.enableLed(xLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.b;
            aCurrState = gamepad1.a;
            xCurrState = gamepad1.x;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor1.enableLed(bLedOn);
            }
            if ((aCurrState == true) && (aCurrState != aPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                aLedOn = !aLedOn;
                colorSensor2.enableLed(aLedOn);
            }
            if ((xCurrState == true) && (xCurrState != xPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                xLedOn = !xLedOn;
                colorSensor3.enableLed(xLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;
            aPrevState = aCurrState;
            xPrevState = xCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor1.red() * 8, colorSensor1.green() * 8, colorSensor1.blue() * 8, hsvValues);
            Color.RGBToHSV(colorSensor2.red() * 8, colorSensor2.green() * 8, colorSensor2.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear1", colorSensor1.alpha());
            telemetry.addData("Red1  ", colorSensor1.red());
            telemetry.addData("Green1", colorSensor1.green());
            telemetry.addData("Blue1 ", colorSensor1.blue());
            telemetry.addData("Hue1", hsvValues[0]);
            telemetry.addData("LED2", aLedOn ? "On" : "Off");
            telemetry.addData("Clear2", colorSensor2.alpha());
            telemetry.addData("Red2  ", colorSensor2.red());
            telemetry.addData("Green2", colorSensor2.green());
            telemetry.addData("Blue2 ", colorSensor2.blue());
            telemetry.addData("Hue2", hsvValues2[0]);
            telemetry.addData("LED3", xLedOn ? "On" : "Off");
            telemetry.addData("Clear3", colorSensor3.alpha());
            telemetry.addData("Red3  ", colorSensor3.red());
            telemetry.addData("Green3", colorSensor3.green());
            telemetry.addData("Blue3 ", colorSensor3.blue());
            telemetry.addData("Hue3", hsvValues3[0]);
            telemetry.addData("i2c1", colorSensor1.getI2cAddress().get8Bit());
            telemetry.addData("i2c2", colorSensor2.getI2cAddress().get8Bit());
            telemetry.addData("i2c3", colorSensor3.getI2cAddress().get8Bit());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
