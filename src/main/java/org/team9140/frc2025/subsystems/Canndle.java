package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.*;
import java.util.concurrent.TimeUnit;

public class Canndle extends SubsystemBase {
    private static Canndle instance;

    public static final Color8Bit RED = new Color8Bit(255, 0, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    public static final Color8Bit PINK = new Color8Bit(245, 110, 229);
    public static final Color8Bit ORANGE = new Color8Bit(255, 255, 255git );
    public static final Color8Bit PURPLE = new Color8Bit(227, 18, 254);
    public static final Color8Bit WHITE = new Color8Bit(255, 255, 0);
    public static final Color8Bit OFF = new Color8Bit(0, 0, 0);

    private CANdle candle;

    private Canndle() {
        this.candle = new CANdle(0);
    }

    public static Canndle getInstance() {
        return (instance == null) ? instance = new Canndle() : instance;
    }

    public Command setColor(Color8Bit color) {
        return this.runOnce(() -> this.candle.setLEDs(color.red, color.green, color.blue)).asProxy();
    }

    public Command solidAllianceColor() {
        return new ConditionalCommand(this.setColor(RED), this.setColor(BLUE),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())).asProxy();
    }

    public Color8Bit getAllianceColor() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            return BLUE;
        } else {
            return RED;
        }
    }

    public Command turnOff() {
        return this.setColor(new Color8Bit(0, 0, 0)).asProxy();
    }

    public Command blinkColorEndsAlliance(Color8Bit color, Time wait, Time timeout) {
        return this.changeColors(color, OFF, wait, timeout, getAllianceColor());
    }

    public Command blinkColorEndsOff(Color8Bit color, Time wait, Time timeout) {
        return this.changeColors(color, OFF, wait, timeout, OFF);
    }

    public Command blinkColorEndsColor(Color8Bit color, Time wait, Time timeout) {
        return this.changeColors(color, OFF, wait, timeout, color);
    }

    public Command blinkColorForever(Color8Bit color, Time wait) {
        return this.changeColorsForever(color, OFF, wait);
    }

    public Command switchTwoColorsEndsAlliance(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, getAllianceColor());
    }

    public Command switchWithAllianceEndsAlliance(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, getAllianceColor());
    }

    public Command switchWithAllianceEndsFirst(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, firstColor);
    }

    public Command switchWithAllianceEndsOff(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, getAllianceColor(), wait, timeout, OFF);
    }

    public Command switchTwoColorsEndsFirst(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, firstColor);
    }

    public Command switchTwoColorsEndsSecond(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, secondColor);
    }

    public Command switchTwoColorsEndsOff(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout) {
        return this.changeColors(firstColor, secondColor, wait, timeout, OFF);
    }

    public Command changeColors(Color8Bit firstColor, Color8Bit secondColor, Time wait, Time timeout, Color8Bit finalColor) {
        return this.setColor(firstColor).andThen(Commands.waitTime(wait)).andThen(setColor(secondColor)).andThen(Commands.waitTime(wait)).repeatedly().withTimeout(timeout).andThen(setColor(finalColor)).asProxy();
    }

    public Command changeColorsForever(Color8Bit firstColor, Color8Bit secondColor, Time wait) {
        return this.setColor(firstColor).andThen(Commands.waitTime(wait)).andThen(setColor(secondColor)).andThen(Commands.waitTime(wait)).repeatedly().asProxy();
    }

    public Command flashColor(Color8Bit color, Time holdTime) {
        return this.setColor(color).andThen(Commands.waitTime(holdTime).andThen(this.turnOff())).asProxy();
    }
}



