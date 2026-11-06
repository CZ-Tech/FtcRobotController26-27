package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.http.PemTls;
import org.firstinspires.ftc.teamcode.common.network.http.HttpResponse;
import org.firstinspires.ftc.teamcode.common.network.http.RobotHttpServer;
import org.junit.After;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.ServerSocket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.security.KeyStore;
import java.security.PrivateKey;
import java.security.cert.Certificate;
import java.security.cert.X509Certificate;
import java.security.interfaces.ECPrivateKey;
import java.util.Arrays;
import java.util.Base64;

import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLContext;
import javax.net.ssl.TrustManager;
import javax.net.ssl.X509TrustManager;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** HTTP and HTTPS are served side by side by the same RobotHttpServer. */
public class HttpsServerTest {
    private static final String HEALTH_BODY = "{\"status\":\"ok\"}";

    private RobotHttpServer server;

    @After
    public void tearDown() {
        if (server != null) server.stop();
    }

    /** Generated PEM material for one test key type. */
    private static final class Pem {
        String fullchain;
        PrivateKey key;
        X509Certificate leaf;
    }

    private static Pem generate(String algorithm, String sizeOrCurveArg) throws Exception {
        File dir = Files.createTempDirectory("https-test").toFile();
        File store = new File(dir, "ks.p12");
        String keytool = new File(System.getProperty("java.home"), "bin/keytool").getPath();
        Process process = new ProcessBuilder(
                keytool, "-genkeypair", "-alias", "t", "-keyalg", algorithm,
                algorithm.equals("EC") ? "-groupname" : "-keysize", sizeOrCurveArg,
                "-dname", "CN=localhost", "-ext", "san=dns:localhost,ip:127.0.0.1",
                "-validity", "2", "-storetype", "PKCS12",
                "-keystore", store.getPath(), "-storepass", "changeit")
                .redirectErrorStream(true)
                .start();
        byte[] output = process.getInputStream().readAllBytes();
        if (process.waitFor() != 0) fail("keytool failed: " + new String(output));

        KeyStore ks = KeyStore.getInstance("PKCS12");
        try (InputStream in = new FileInputStream(store)) {
            ks.load(in, "changeit".toCharArray());
        }
        Pem pem = new Pem();
        pem.key = (PrivateKey) ks.getKey("t", "changeit".toCharArray());
        Certificate[] chain = ks.getCertificateChain("t");
        pem.leaf = (X509Certificate) chain[0];
        StringBuilder text = new StringBuilder();
        for (Certificate certificate : chain) {
            text.append(armor("CERTIFICATE", certificate.getEncoded()));
        }
        pem.fullchain = text.toString();
        return pem;
    }

    private static String armor(String label, byte[] der) {
        return "-----BEGIN " + label + "-----\n"
                + Base64.getMimeEncoder(64, "\n".getBytes(StandardCharsets.US_ASCII)).encodeToString(der)
                + "\n-----END " + label + "-----\n";
    }

    /** Strips the PKCS#8 wrapper, leaving the inner PKCS#1 / SEC1 structure. */
    private static byte[] pkcs8Inner(byte[] pkcs8) {
        // SEQ { INT 0, SEQ algId, OCTET STRING inner }
        int[] top = tlv(pkcs8, 0);
        int[] version = tlv(pkcs8, top[1]);
        int[] algorithm = tlv(pkcs8, version[2]);
        int[] octets = tlv(pkcs8, algorithm[2]);
        return Arrays.copyOfRange(pkcs8, octets[1], octets[2]);
    }

    private static int[] tlv(byte[] d, int off) {
        int len = d[off + 1] & 0xFF;
        int start = off + 2;
        if (len >= 0x80) {
            int n = len & 0x7F;
            len = 0;
            for (int i = 0; i < n; i++) len = (len << 8) | (d[start + i] & 0xFF);
            start += n;
        }
        return new int[] {d[off] & 0xFF, start, start + len};
    }

    /** Builds a SEC1 "EC PRIVATE KEY" naming secp256r1, as certbot/openssl write it. */
    private static byte[] sec1P256(ECPrivateKey key) {
        byte[] s = key.getS().toByteArray();
        byte[] raw = new byte[32];
        int copy = Math.min(32, s.length);
        System.arraycopy(s, s.length - copy, raw, 32 - copy, copy);
        byte[] oid = {0x06, 0x08, 0x2A, (byte) 0x86, 0x48, (byte) 0xCE, 0x3D, 0x03, 0x01, 0x07};
        byte[] body = concat(
                new byte[] {0x02, 0x01, 0x01},
                new byte[] {0x04, 0x20}, raw,
                new byte[] {(byte) 0xA0, (byte) oid.length}, oid);
        return concat(new byte[] {0x30, (byte) body.length}, body);
    }

    private static byte[] concat(byte[]... parts) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        for (byte[] part : parts) out.write(part, 0, part.length);
        return out.toByteArray();
    }

    private static int freePort() throws Exception {
        try (ServerSocket probe = new ServerSocket(0)) {
            return probe.getLocalPort();
        }
    }

    private static SSLContext trusting(X509Certificate expected) throws Exception {
        TrustManager manager = new X509TrustManager() {
            @Override
            public void checkClientTrusted(X509Certificate[] chain, String type) {}

            @Override
            public void checkServerTrusted(X509Certificate[] chain, String type) {
                assertEquals(expected, chain[0]);
            }

            @Override
            public X509Certificate[] getAcceptedIssuers() {
                return new X509Certificate[0];
            }
        };
        SSLContext context = SSLContext.getInstance("TLS");
        context.init(null, new TrustManager[] {manager}, null);
        return context;
    }

    private static String get(String url, SSLContext tls) throws Exception {
        HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
        if (tls != null) {
            ((HttpsURLConnection) connection).setSSLSocketFactory(tls.getSocketFactory());
            ((HttpsURLConnection) connection).setHostnameVerifier((host, session) -> true);
        }
        connection.setConnectTimeout(3000);
        connection.setReadTimeout(3000);
        assertEquals(200, connection.getResponseCode());
        try (InputStream in = connection.getInputStream()) {
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    private void serveAndCheck(Pem pem, String keyPem) throws Exception {
        PemTls.Material material = PemTls.load(pem.fullchain, keyPem);
        assertEquals(pem.leaf, material.leaf);

        int httpPort = freePort();
        server = new RobotHttpServer(httpPort, exchange ->
                exchange.send(HttpResponse.json(200, HEALTH_BODY)));
        server.enableTls(0, material.context);
        server.start();

        assertTrue(server.tlsPort() > 0);
        assertEquals(HEALTH_BODY, get("http://127.0.0.1:" + httpPort + "/x", null));
        assertEquals(HEALTH_BODY,
                get("https://127.0.0.1:" + server.tlsPort() + "/x", trusting(pem.leaf)));
    }

    @Test
    public void rsaPkcs8() throws Exception {
        Pem pem = generate("RSA", "2048");
        serveAndCheck(pem, armor("PRIVATE KEY", pem.key.getEncoded()));
    }

    @Test
    public void rsaPkcs1() throws Exception {
        Pem pem = generate("RSA", "2048");
        serveAndCheck(pem, armor("RSA PRIVATE KEY", pkcs8Inner(pem.key.getEncoded())));
    }

    @Test
    public void ecPkcs8() throws Exception {
        Pem pem = generate("EC", "secp256r1");
        serveAndCheck(pem, armor("PRIVATE KEY", pem.key.getEncoded()));
    }

    @Test
    public void ecSec1() throws Exception {
        Pem pem = generate("EC", "secp256r1");
        serveAndCheck(pem, armor("EC PRIVATE KEY", sec1P256((ECPrivateKey) pem.key)));
    }

    @Test
    public void keyThatDoesNotMatchCertificateIsRejectedOrFailsHandshake() throws Exception {
        Pem a = generate("RSA", "2048");
        Pem b = generate("RSA", "2048");
        try {
            PemTls.Material material = PemTls.load(a.fullchain, armor("PRIVATE KEY", b.key.getEncoded()));
            int httpPort = freePort();
            server = new RobotHttpServer(httpPort, exchange ->
                    exchange.send(HttpResponse.json(200, HEALTH_BODY)));
            server.enableTls(0, material.context);
            server.start();
            // HTTP must still work even though HTTPS is unusable.
            assertEquals(HEALTH_BODY, get("http://127.0.0.1:" + httpPort + "/x", null));
            try {
                get("https://127.0.0.1:" + server.tlsPort() + "/x", trusting(a.leaf));
                fail("handshake with mismatched key must not succeed");
            } catch (java.io.IOException expected) {
                // fine
            }
        } catch (java.security.GeneralSecurityException | java.io.IOException rejected) {
            // Also acceptable: the mismatch is detected while loading.
        }
    }

    @Test
    public void garbageMaterialIsReportedNotSwallowed() {
        try {
            PemTls.load("not a pem", "also not a pem");
            fail("expected failure");
        } catch (Exception expected) {
            assertNotNull(expected.getMessage());
        }
    }

    @Test
    public void httpKeepsWorkingWhenTlsPortIsTaken() throws Exception {
        Pem pem = generate("RSA", "2048");
        PemTls.Material material = PemTls.load(pem.fullchain, armor("PRIVATE KEY", pem.key.getEncoded()));
        try (ServerSocket occupied = new ServerSocket(0)) {
            int httpPort = freePort();
            server = new RobotHttpServer(httpPort, exchange ->
                    exchange.send(HttpResponse.json(200, HEALTH_BODY)));
            server.enableTls(occupied.getLocalPort(), material.context);
            server.start();
            assertEquals(-1, server.tlsPort());
            assertEquals(HEALTH_BODY, get("http://127.0.0.1:" + httpPort + "/x", null));
        }
    }
}
