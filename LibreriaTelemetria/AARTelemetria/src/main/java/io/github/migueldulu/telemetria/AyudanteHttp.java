package io.github.migueldulu.telemetria;

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.concurrent.*;
import android.os.Looper;
import android.util.Log;

// Small HTTP helper used from native code via JNI.
// It exposes a single static entry point (makeRequest) that performs
// an HTTP request with HttpURLConnection and returns the raw response
// bytes. If called on the main thread, it offloads the work to a
// background executor to avoid NetworkOnMainThreadException.
public class AyudanteHttp {

    // Main entry point called from C++ (via AndroidUploader).
    // - method: HTTP method ("GET", "POST", etc).
    // - url: full URL as a String.
    // - body: request body as bytes (for example JSON).
    // - headers: HTTP headers key -> value.
    public static byte[] makeRequest(String method, String url, byte[] body, Map<String,String> headers) {
        if (Looper.getMainLooper() == Looper.myLooper()) {
            // Called on main thread -> offload to executor
            return runInWorker(method, url, body, headers);
        } else {
            // Already in a background thread -> perform request directly
            return doRequest(method, url, body, headers);
        }
    }

    // Runs doRequest(...) on a background executor.
    // This is used when makeRequest is invoked on the main thread, so that
    // we do not perform network input/output on the UserInterface thread.
    private static byte[] runInWorker(String method, String url, byte[] body, Map<String,String> headers) {
        ExecutorService ex = Executors.newSingleThreadExecutor();
        Future<byte[]> fut = ex.submit(() -> doRequest(method, url, body, headers));
        try {
            // Wait at most 60 seconds for the HTTP request to complete
            return fut.get(60, TimeUnit.SECONDS);
        } catch (Exception e) {
            Log.e("telemetria", "Worker exception", e);
            return errorBytes(e);
        } finally {
            ex.shutdownNow();
        }
    }

    // Performs the actual HTTP request using HttpURLConnection. If the HTTP
    // status code is not in the 2xx range, it appends a trailing line "HTTP_STATUS:<code>" to the response body.
    private static byte[] doRequest(String method, String url, byte[] body, Map<String,String> headers) {
        try {
            HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setRequestMethod(method);
            conn.setConnectTimeout(15000);
            conn.setReadTimeout(30000);
            conn.setUseCaches(false);
            conn.setDoInput(true);

            // Apply all provided headers to the request.
            if (headers != null) {
                for (Map.Entry<String,String> e : headers.entrySet()) {
                    conn.setRequestProperty(e.getKey(), e.getValue());
                }
            }
            // If a request body is provided, write it to the output stream.
            if (body != null && body.length > 0) {
                conn.setDoOutput(true);
                try (OutputStream os = conn.getOutputStream()) { os.write(body); }
            }
            // Perform the request and obtain the HTTP status code.
            int code = conn.getResponseCode();
            // For 2xx codes, use getInputStream(); otherwise, use getErrorStream()
            InputStream is = (code >= 200 && code < 300) ? conn.getInputStream() : conn.getErrorStream();
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            if (is != null) {
                try {
                    byte[] buf = new byte[8192];
                    int n;
                    // Read the response body into memory.
                    while ((n = is.read(buf)) > 0) bos.write(buf, 0, n);
                } finally {
                    try { is.close(); } catch (IOException ignore) {}
                }
            }
            // If the HTTP status is not 2xx, append an indicator line at the end
            if (code < 200 || code >= 300) {
                bos.write(("\nHTTP_STATUS:" + code).getBytes());
            }
            return bos.toByteArray();
        } catch (Exception ex) {
            Log.e("telemetria", "AyudanteHttp exception", ex);
            return errorBytes(ex);
        }
    }

    // Converts an Exception into a small JSON error object:
    private static byte[] errorBytes(Exception ex) {
        String msg = "{\"error\":\"" + ex.getClass().getSimpleName() + "\",\"msg\":\"" +
                (ex.getMessage() == null ? "" : ex.getMessage()) + "\"}";
        return msg.getBytes();
    }
}
