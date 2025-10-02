package io.github.migueldulu.telemetria;

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.concurrent.*;
import android.os.Looper;
import android.util.Log;

public class AyudanteHttp {

    public static byte[] makeRequest(String method, String url, byte[] body, Map<String,String> headers) {
        if (Looper.getMainLooper() == Looper.myLooper()) {
            return runInWorker(method, url, body, headers);
        } else {
            return doRequest(method, url, body, headers);
        }
    }

    private static byte[] runInWorker(String method, String url, byte[] body, Map<String,String> headers) {
        ExecutorService ex = Executors.newSingleThreadExecutor();
        Future<byte[]> fut = ex.submit(() -> doRequest(method, url, body, headers));
        try {
            return fut.get(60, TimeUnit.SECONDS);
        } catch (Exception e) {
            Log.e("telemetria", "Worker exception", e);
            return errorBytes(e);
        } finally {
            ex.shutdownNow();
        }
    }

    private static byte[] doRequest(String method, String url, byte[] body, Map<String,String> headers) {
        try {
            HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setRequestMethod(method);
            conn.setConnectTimeout(15000);
            conn.setReadTimeout(30000);
            conn.setUseCaches(false);
            conn.setDoInput(true);

            if (headers != null) {
                for (Map.Entry<String,String> e : headers.entrySet()) {
                    conn.setRequestProperty(e.getKey(), e.getValue());
                }
            }
            if (body != null && body.length > 0) {
                conn.setDoOutput(true);
                try (OutputStream os = conn.getOutputStream()) { os.write(body); }
            }

            int code = conn.getResponseCode();
            InputStream is = (code >= 200 && code < 300) ? conn.getInputStream() : conn.getErrorStream();
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            if (is != null) {
                try {
                    byte[] buf = new byte[8192];
                    int n;
                    while ((n = is.read(buf)) > 0) bos.write(buf, 0, n);
                } finally {
                    try { is.close(); } catch (IOException ignore) {}
                }
            }
            if (code < 200 || code >= 300) {
                bos.write(("\nHTTP_STATUS:" + code).getBytes());
            }
            return bos.toByteArray();
        } catch (Exception ex) {
            Log.e("telemetria", "AyudanteHttp exception", ex);
            return errorBytes(ex);
        }
    }

    private static byte[] errorBytes(Exception ex) {
        String msg = "{\"error\":\"" + ex.getClass().getSimpleName() + "\",\"msg\":\"" +
                (ex.getMessage() == null ? "" : ex.getMessage()) + "\"}";
        return msg.getBytes();
    }
}
