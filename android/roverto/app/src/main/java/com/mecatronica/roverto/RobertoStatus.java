package com.mecatronica.roverto;

class RovertoStatus {
    static public final int MESSAGE_SIZE = 16;
    public int[] data = new int[MESSAGE_SIZE];

    public void fromByteArray(byte[] array) throws BadBufferSize {
        if (array.length != MESSAGE_SIZE) throw new BadBufferSize("Message bad buffer size");
        for (int i = 0; i < MESSAGE_SIZE; ++i)
            data[i] = array[i]<0? ((int) array[i]) + 128 : ((int) array[i]);
    }

    static public RovertoStatus getFromByteArray(byte[] array) throws BadBufferSize
    {
        RovertoStatus msg = new RovertoStatus();
        msg.fromByteArray(array);
        return msg;
    }
}

class BadBufferSize extends Exception {
    public BadBufferSize(String message) {
        super(message);
    }
}

