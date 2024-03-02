package frc.lib.util;

import edu.wpi.first.wpilibj.AnalogInput;


public class Resolver{

    public int address = 0;
    public double offset = 0;
    public AnalogInput channel;
    public double value = 0;
    public double staticoffset = 0;

    public Resolver(int address) {
        this.address = address;
        this.channel = new AnalogInput(address);
        this.offset = channel.getVoltage();
    }

    public void update(){
       this.value = channel.getVoltage() * 72;
       this.value = this.value - offset;
       this.value = this.value - staticoffset;
       if (this.value <0) {this.value = this.value+360;}
    }



    public void zero(){
        this.offset = channel.getVoltage();
    }
        
    }
