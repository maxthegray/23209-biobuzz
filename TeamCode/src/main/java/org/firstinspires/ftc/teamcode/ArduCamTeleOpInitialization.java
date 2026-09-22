import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class ArduCamTeleOpInitialization {
    static {
        // Load the OpenCV native library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args) {
        // Open the camera (usually index 0 or 1 depending on internal webcams)
        VideoCapture camera = new VideoCapture(0);

        if (!camera.isOpened()) {
            System.err.println("Error: Could not open Arducam device.");
            return;
        }

        // 1. Force MJPEG compression (Crucial for achieving 100 FPS on USB 2.0)
        // 'M', 'J', 'P', 'G' translates to fourcc integer value 1196444237
        camera.set(Videoio.CAP_PROP_FOURCC, Videoio.VideoWriter_fourcc('M', 'J', 'P', 'G'));

        // 2. Set Resolution supported by the 100 FPS profile (1280 x 800)
        camera.set(Videoio.CAP_PROP_FRAME_WIDTH, 1280);
        camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, 800);

        // 3. Request 100 FPS
        camera.set(Videoio.CAP_PROP_FPS, 100);

        // Verify the settings applied successfully
        double actualWidth = camera.get(Videoio.CAP_PROP_FRAME_WIDTH);
        double actualHeight = camera.get(Videoio.CAP_PROP_FRAME_HEIGHT);
        double actualFPS = camera.get(Videoio.CAP_PROP_FPS);

        System.out.println("Arducam Initialized Successfully!");
        System.out.printf("Resolution: %.0fx%.0f\n", actualWidth, actualHeight);
        System.out.printf("Target Frame Rate: %.2f FPS\n", actualFPS);

        // Frame processing loop
        Mat frame = new Mat();
        long startTime = System.currentTimeMillis();
        int frameCount = 0;

        System.out.println("Starting high-speed frame capture loop. Press Ctrl+C to stop...");

        while (true) {
            // Read a frame from the camera buffer
            if (camera.read(frame)) {
                frameCount++;

                // Do your real-time machine vision processing here (e.g., ArUco, tracking)

                // Benchmark every 100 frames to monitor actual hardware performance
                if (frameCount % 100 == 0) {
                    long endTime = System.currentTimeMillis();
                    double elapsedSeconds = (endTime - startTime) / 1000.0;
                    double currentFPS = frameCount / elapsedSeconds;
                    System.out.printf("Actual processing speed: %.2f FPS\n", currentFPS);
                }
            } else {
                System.err.println("Warning: Frame grab failed.");
            }
        }
    }
}
