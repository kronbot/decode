package org.firstinspires.ftc.teamcode.kronbot.manual;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.kronbot.baseOps.MainTeleOp;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

//@TeleOp(name = "Example", group = Constants.MAIN_GROUP)
public class OpModeWrapperExample extends OpMode {

    MainTeleOp OpMode = new MainTeleOp();

    public void init() {
        OpMode.init();
        //OpMode.ConfigVariable = smthidk;
    }

    public void init_loop() {
        OpMode.init_loop();
    }

    public void start() {
        OpMode.start();
    }

    public void loop() {
        OpMode.loop();
    }

    public void stop() {
        OpMode.stop();
    }

}