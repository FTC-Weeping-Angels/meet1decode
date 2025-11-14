package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;

public abstract class OpModeCommand extends OpMode {

    public void reset() {CommandScheduler.getInstance().reset();}
    public void run() {CommandScheduler.getInstance().run();}

    public void register(Subsystem... subsystems) {CommandScheduler.getInstance().registerSubsystem(subsystems);}
    public void schedule(Command... commands) {CommandScheduler.getInstance().schedule(commands);}

    @Override
    public void init() {initialize();}

    @Override
    public void loop() {run();}

    public void stop() {reset();}

    public abstract void initialize();
}
