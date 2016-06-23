package com.mecatronica.arduinoadk;

import java.io.IOException;

/**
 * Base class for USB Communication
 */
public interface UsbCommunicationInterface extends Runnable
{
    byte get();

    void send(byte data) throws IOException;

    void cancel();
}
