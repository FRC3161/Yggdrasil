package com.team254.lib;

/**
 * @author Tom Bottiglieri Team 254, The Cheesy Poofs
 */
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * @deprecated as of 2015 Recycle Rush season, no interaction with the robot is allowed
 * during autonomous, not even indirect control using the Kinect or solutions like
 * Cheesy Vision.
 */
@Deprecated
public enum CheesyVisionServer {

    INSTANCE;

    private static final ExecutorService THREAD_POOL = Executors.newCachedThreadPool();
    public static final int DEFAULT_PORT = 1180;
    public static final double CLIENT_TIMEOUT_SECONDS = 3.0;
    private final List<Socket> connections = new ArrayList<>();
    private volatile boolean counting = false, listening = true, curLeftStatus = false, curRightStatus = false;
    private volatile int leftCount = 0, rightCount = 0, totalCount = 0;
    private volatile double lastHeartbeatTime = -1.0d;
    private volatile Future<?> serverJob;

    public void start() {
        if (serverJob != null) {
            serverJob.cancel(true);
        }
        serverJob = THREAD_POOL.submit(new ServerTask());
    }

    public void stop() {
        listening = false;
        serverJob.cancel(true);
        THREAD_POOL.shutdownNow();
        final Iterator<Socket> socketIterator = connections.iterator();
        while (socketIterator.hasNext()) {
            final Socket socket = socketIterator.next();
            try {
                socket.close();
            } catch (final IOException e) {
                e.printStackTrace();
            }
            socketIterator.remove();
        }
    }

    public boolean hasClientConnection() {
        return lastHeartbeatTime > 0 && (Timer.getFPGATimestamp() - lastHeartbeatTime) < CLIENT_TIMEOUT_SECONDS
                && !connections.isEmpty();
    }

    private void updateCounts(final boolean left, final boolean right) {
        if (!counting) {
            return;
        }
        if (left) {
            ++leftCount;
        }
        if (right) {
            ++rightCount;
        }
        ++totalCount;
    }

    public void startSamplingCounts() {
        counting = true;
    }

    public void stopSamplingCounts() {
        counting = false;
    }

    public void reset() {
        leftCount = 0;
        rightCount = 0;
        curLeftStatus = false;
        curRightStatus = false;
    }

    public int getLeftCount() {
        return leftCount;
    }

    public int getRightCount() {
        return rightCount;
    }

    public int getTotalCount() {
        return totalCount;
    }

    public boolean getLeftStatus() {
        return curLeftStatus;
    }

    public boolean getRightStatus() {
        return curRightStatus;
    }

    // This class handles incoming TCP connections
    private class VisionServerConnectionHandler implements Runnable {
        public static final int BUFFER_SIZE = 1024;
        public static final int SLEEP_PERIOD = 50;
        private final Socket connection;
        public VisionServerConnectionHandler(final Socket c) {
            connection = c;
        }

        public void run() {
            try (final InputStream is = connection.getInputStream()) {
                final byte[] b = new byte[BUFFER_SIZE];
                final double timeout = 10.0d;
                double lastHeartbeat = Timer.getFPGATimestamp();
                lastHeartbeatTime = lastHeartbeat;
                while (Timer.getFPGATimestamp() < lastHeartbeat + timeout) {
                    while (is.available() > 0) {
                        final int read = is.read(b);
                        for (int i = 0; i < read; i++) {
                            final byte reading = b[i];
                            final boolean leftStatus = (reading & (1 << 1)) > 0;
                            final boolean rightStatus = (reading & 1) > 0;
                            curLeftStatus = leftStatus;
                            curRightStatus = rightStatus;
                            updateCounts(leftStatus, rightStatus);
                        }
                        lastHeartbeat = Timer.getFPGATimestamp();
                        lastHeartbeatTime = lastHeartbeat;
                    }
                    try {
                        Thread.sleep(SLEEP_PERIOD);
                    } catch (final InterruptedException ex) {
                        System.out.println("Thread sleep failed.");
                    }
                }
            } catch (final IOException e) {
                e.printStackTrace();
            } finally {
                try {
                    connection.close();
                } catch (final IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private class ServerTask implements Runnable {
        public void run() {
            try {
                final ServerSocket s = new ServerSocket(DEFAULT_PORT);
                while (listening) {
                    final Socket connection = s.accept();
                    THREAD_POOL.submit(new VisionServerConnectionHandler(connection));
                    connections.add(connection);
                }
            } catch (final IOException e) {
                System.out.println("Socket failure.");
                e.printStackTrace();
            }
        }
    }
}
