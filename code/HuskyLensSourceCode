package org.firstinspires.ftc.teamcode.HuskyLens;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@I2cDeviceType
@DeviceProperties(name = "HuskyLens vision sensor", description = "HuskyLens", xmlTag = "HUSKYLENS")
public class HuskyLens extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    
    public final byte ALL = 0x20;
    public final byte BLOCKS = 0x21;
    public final byte ARROWS = 0x22;
    public final byte LEARNED = 0x23;
    public final byte BLOCKS_LEARNED = 0x24;
    public final byte ARROWS_LEARNED = 0x25;
    public final byte BY_ID = 0x26;
    public final byte BLOCKS_BY_ID = 0x27;
    public final byte ARROWS_BY_ID = 0x28;
    
    public final byte FACE_RECOGNITION = 0x00;
    public final byte OBJECT_TRACKING = 0x01;
    public final byte OBJECT_RECOGNITION = 0x02;
    public final byte LINE_TRACKING = 0x03;
    public final byte COLOR_RECOGNITION = 0x04;
    public final byte TAG_RECOGNITION = 0x05;
    public final byte OBJECT_CLASSIFICATION = 0x06;
    
    private int bytetoshort(byte[] m, int i) {
        return (m[i]&0xff) + ((m[i+1]&0xff)<<8);
    }
    
    public class Block {
        public byte[] msg = null;
        public int center() { return bytetoshort(msg, 5); }
        public int middle() { return bytetoshort(msg, 7); }
        public int width() { return bytetoshort(msg,9); }
        public int height() { return bytetoshort(msg,11); }
        public int top() { return middle() - height()/2; }
        public int left() { return center() - width()/2; }
        public int id() { return bytetoshort(msg,13); }
        public Block(byte[] m) { msg = m; }
        public String toString() {
            return "id=" + id() + " " + width() + "x" + height() + "@" + center() + "," + middle();
        }
    }
    
    public Block blocks[] = null;

    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    
    public synchronized boolean doInitialize() {
        return true;
    }
    
    public String getDeviceName() {
        return "HuskyLens vision sensor";
    }
    
    public HuskyLens(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        I2cDeviceSynch.ReadWindow readWindow = 
            new I2cDeviceSynch.ReadWindow(0, 26, I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x32));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    
    public void cmdRequest(byte t) {
        byte[] frame = { 0x55, (byte)0xaa, 0x11, 0x00, 0x20, 0x30 };
        frame[4] = t;
        writeRequest(frame);
        readBlocks();
    }
    
    public void cmdRequest(byte t, int id) {
        byte[] frame = { 0x55, (byte)0xaa, 0x11, 0x02, 0x26, 0x00, 0x00, 0x39};
        frame[4] = t;
        frame[5] = (byte)(id & 0xff);
        frame[6] = (byte)(id >> 8);
        writeRequest(frame);
        readBlocks();
    }
    
    public void cmdRequestAlgorithm(byte t) {
        byte[] frame = { 0x55, (byte)0xaa, 0x11, 0x02, 0x2d, 0x01, 0x00, 0x40};
        frame[5] = t;
        frame[6] = 0;
        writeRequest(frame);
        byte[] ok = this.deviceClient.read(0,6);
    }

    private void writeRequest(byte[] frame) {
        int l = frame.length;
        int checksum = 0;
        for (int i = 0; i < l - 1; i++) checksum += (frame[i] & 0xff);
        frame[l-1] = (byte)(checksum & 0xff);
        this.deviceClient.write(frame);
    }

    private void readBlocks() {
        byte[] info = this.deviceClient.read(0,15);
        int count = bytetoshort(info, 5);
        blocks = new Block[count];
        for (int i = 0; i < count; i++) {
            blocks[i] = new Block(this.deviceClient.read(0,15));
        }
    }    

}
