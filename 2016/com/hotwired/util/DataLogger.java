package com.revAmped.util;

/**
 * Created by Olavi Kamppari on 9/9/2015.
 */
 import java.io.File;
 import java.io.FileWriter;
 import java.io.IOException;
 import java.io.Writer;

public class DataLogger {
    private Writer writer;
    private StringBuffer lineBuffer;
    private long msBase;
    private long nsBase;

    public DataLogger (String fileName) {
        String directoryPath    = "/sdcard/FIRST/DataLogger";
        String filePath         = directoryPath + "/" + fileName + ".csv";

        new File(directoryPath).mkdir();        // Make sure that the directory exists

        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
        }
        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        add("sec");
        add("d ms");
    }

    private void flush(){
        long milliTime,nanoTime;

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            lineBuffer.setLength(0);
        }
        catch (IOException e){
        }
        milliTime   = System.currentTimeMillis();
        nanoTime    = System.nanoTime();
        add(String.format("%.3f", (milliTime - msBase) / 1.0E3));
        add(String.format("%.3f", (nanoTime - nsBase) / 1.0E6));
        nsBase      = nanoTime;
    }

    public void close() {
        try {
            writer.close();
        }
        catch (IOException e) {
        }
    }

    public void add(String s) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }

    public void add(char c) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }

    public void add(boolean b) {
        add(b ? '1' : '0');
    }

    public void add(byte b) {
        add(Byte.toString(b));
    }

    public void add(short s) {
        add(Short.toString(s));
    }

    public void add(long l) {
        add(Long.toString(l));
    }

    public void add(float f) {
        add(Float.toString(f));
    }

    public void add(double d) {
        add(Double.toString(d));
    }

    public void newLine() {
        flush();
    }

    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }
}