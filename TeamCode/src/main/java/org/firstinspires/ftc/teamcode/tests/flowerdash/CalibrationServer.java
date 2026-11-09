package org.firstinspires.ftc.teamcode.tests.flowerdash;

import android.content.Context;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import fi.iki.elonen.NanoHTTPD;

// tiny web server just for this test. It is its own server on its own port and never touches the robot controller's
// web server (8080) or the Pedro Pathing Panels dashboard (8001), and it shuts down when the opmode ends.
// No login: it can only be reached from the robot's own network.
public class CalibrationServer extends NanoHTTPD {

    // none of these are 8001 (Panels) or 8080 (robot controller)
    private static final int[] PORTS = {8765, 8766, 8767, 8768, 8769, 8770};

    private final Context context;
    private final CalibrationSession session;
    private final Queue<Command> commands;
    private String page;

    private CalibrationServer(int port, Context context, CalibrationSession session, Queue<Command> commands) {
        super(port);
        this.context = context;
        this.session = session;
        this.commands = commands;
    }

    // tries each port in turn, returns null if none work. The test still runs fine without the website
    public static CalibrationServer startOnFreePort(Context context, CalibrationSession session, Queue<Command> commands) {
        for (int port : PORTS) {
            CalibrationServer server = new CalibrationServer(port, context, session, commands);
            try {
                server.start(NanoHTTPD.SOCKET_READ_TIMEOUT, true);
                return server;
            } catch (IOException | RuntimeException e) {
                server.stop();
            }
        }
        return null;
    }

    public int port() {
        return getListeningPort();
    }

    @Override
    public Response serve(IHTTPSession http) {
        try {
            String uri = http.getUri();
            if (uri.equals("/") || uri.equals("/index.html")) return html();
            if (uri.equals("/api/state")) {
                int since = 0;
                String s = http.getParms().get("since");
                if (s != null) since = Integer.parseInt(s);
                return json(session.snapshotJson(since));
            }
            if (uri.equals("/api/cmd") && http.getMethod() == Method.POST) return command(http);
            if (uri.equals("/api/raw.csv")) return csv("flower_raw.csv", session.rawCsv());
            if (uri.equals("/api/summary.csv")) return csv("flower_summary.csv", session.summaryCsv());
            if (uri.equals("/api/fit.csv")) return csv("flower_fit.csv", session.fitCsv());
            if (uri.equals("/api/trials.csv")) return csv("flower_trials.csv", session.trialsCsv());
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "not found");
        } catch (Exception e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", String.valueOf(e));
        }
    }

    private Response command(IHTTPSession http) throws Exception {
        Map<String, String> files = new HashMap<>();
        http.parseBody(files);
        JSONObject body = new JSONObject(files.get("postData"));
        String name = body.optString("cmd");
        String why = session.validate(name);
        if (why != null) {
            return json("{\"ok\":false,\"reason\":\"" + why.replace("\"", "'") + "\"}");
        }
        commands.add(new Command(name, body));
        return json("{\"ok\":true}");
    }

    private Response html() throws IOException {
        if (page == null) {
            try (InputStream in = context.getAssets().open("flower_dash/index.html")) {
                ByteArrayOutputStream out = new ByteArrayOutputStream();
                byte[] buf = new byte[4096];
                int n;
                while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
                page = out.toString("UTF-8");
            }
        }
        Response r = newFixedLengthResponse(Response.Status.OK, "text/html; charset=utf-8", page);
        r.addHeader("Cache-Control", "no-store");
        return r;
    }

    private Response json(String text) {
        Response r = newFixedLengthResponse(Response.Status.OK, "application/json", text);
        r.addHeader("Cache-Control", "no-store");
        return r;
    }

    private Response csv(String filename, String text) {
        Response r = newFixedLengthResponse(Response.Status.OK, "text/csv", text);
        r.addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
        return r;
    }
}
