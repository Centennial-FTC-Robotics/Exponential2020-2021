package org.exponential.utility;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class LogMaker {
    FileWriter fileWriter;
    public LogMaker(String filename) {
        File file = new File(filename);
        try {
            file.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            fileWriter = new FileWriter(filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void write(String str) {
        try {
            fileWriter.write(str);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void close() {
        try {
            if (fileWriter != null) {
                fileWriter.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
