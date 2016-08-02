package com.mecatronica.roverto;

import android.util.Log;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Class for basic byte-based communication through USB
 */
public class SerialUsbCommunication {
    // Debug flag
    public static final boolean D = true;
    // TAG is used to debug in Android console
    private static final String TAG = "roverto";

    // File streams
    private DataInputStream usbInputStream;
    private FileOutputStream usbOutputStream;
    // Queues
    public BlockingQueue<Integer> receivedQueue = new LinkedBlockingQueue<Integer>();
    // Parser
    private PacketParser parser;

    // Max value
    public static final int MIN_VALUE = 0;
    public static final int MAX_VALUE = 255;

    SerialUsbCommunication(FileDescriptor fd)
    {
        // Set streams
        this.usbInputStream = new DataInputStream(new FileInputStream(fd));
        this.usbOutputStream = new FileOutputStream(fd);
        // Receive parser
        parser = new PacketParser(this.receivedQueue, this.usbInputStream);
        parser.start();
    }

    public int available()
    {
        return receivedQueue.size();
    }


    public int read()
    {
        try {
            Integer data = this.receivedQueue.poll(1, TimeUnit.MILLISECONDS);

            if (data != null) {
                Log.d(this.getClass().getSimpleName(), "Got result: " + data.toString());
                return data.intValue();
            }
        } catch (InterruptedException e) {
            Log.d(TAG, "InterruptedException handling received packet.");
        }
        return -1;
    }

    public void write(int data)
    {
        // Saturation
        data = data<MIN_VALUE ? MIN_VALUE : (data>MAX_VALUE ? MAX_VALUE : data);
        try {
            Log.d(TAG, "Write data " + Integer.toString(data));
            usbOutputStream.write(data);
            usbOutputStream.flush();
        } catch (IOException e) {
            Log.d(TAG, "IOException sending packet.");
            e.printStackTrace();
        }
    }

    public void close()
    {
        // Close socket thread
        parser.interrupt();
        try
        {
            if (parser != null)
                parser.cancel();
        }
        catch (Exception ignored) {}
        finally {
            parser = null;
        }
        // Close input stream
        try
        {
            if (usbInputStream != null)
                usbInputStream.close();
        }
        catch (Exception ignored) {}
        finally {
            usbInputStream = null;
        }
        // Close output stream
        try
        {
            if (usbOutputStream != null)
                usbOutputStream.close();
        }
        catch (Exception ignored) {}
        finally {
            usbOutputStream = null;
        }
    }

}
