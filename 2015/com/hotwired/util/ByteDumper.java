package com.revAmped.util;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

/**
 * Persist object to the disk
 */
public class ByteDumper {
    private ObjectOutputStream oout = null;
    private String fileName;

    public ByteDumper(String fileName)
        throws IOException {
        this.fileName = fileName;

        File directoryPath = new File("/sdcard/FIRST/DataLogger");
        File filePath = new File(directoryPath, fileName + ".ser");

        directoryPath.mkdir();        // Make sure that the directory exists

        try {
            FileOutputStream fout = new FileOutputStream(filePath);
            oout = new ObjectOutputStream(fout);
        } catch (IOException e) {
            HwLog.error("Failed to create file " + filePath, e);
            throw e;
        }
    }

    public void close() {
        if (this.oout != null) {
            try {
                this.oout.close();
            } catch (IOException e) {
                // no opt
            }
        }
    }

    public void add(byte[] bytes)
        throws IOException {
        if (bytes == null || this.oout == null) {
            return;
        }
        try {
            this.oout.write(bytes);
            this.oout.flush();
        } catch (IOException e) {
            HwLog.error("Failed to write byte arrsy " + bytes, e);
            throw e;
        }
    }
}
