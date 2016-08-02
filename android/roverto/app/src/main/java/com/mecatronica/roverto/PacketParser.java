package com.mecatronica.roverto;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.NoSuchElementException;
import java.util.concurrent.BlockingQueue;

import android.util.Log;

class PacketParser extends Thread {

    private InputStreamReader input;
    private BlockingQueue<Integer> packetsReceivedQueue;
    boolean running;
    private static final String TAG = "roverto";


    PacketParser(BlockingQueue<Integer> packetsReceivedQueue, InputStream theInput){
        this.input = new InputStreamReader(theInput);
        this.packetsReceivedQueue = packetsReceivedQueue;
        this.running = true;
    }


    @Override
    public synchronized void run() {

        Log.d(this.getClass().getSimpleName(), "Parser started.");

        while (running) {
            try {
                // Put element
                packetsReceivedQueue.put(getNextPacket());
            } catch (IOException e) {
                Log.d(TAG, "IOException while getting/putting next packet.");
                break;
            } catch (InterruptedException e) {
                Log.d(TAG, "InterruptedException while getting/putting next packet.");
                break;
            } catch (NoSuchElementException e) {
                Log.d(TAG,
                        "NoSuchElementException while getting/putting next packet (probably due to disconnect).");
                break;
            }
        }
    }

    void cancel()
    {
        this.running = false;
    }


    private Integer getNextPacket() throws IOException {
        // Raw format
        Integer data = new Integer(input.read());
        Log.d(TAG, "DATA: " + data.toString());
        return data;
    }
}
