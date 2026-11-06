package org.firstinspires.ftc.teamcode.common.network.http;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.security.GeneralSecurityException;
import java.security.KeyFactory;
import java.security.KeyStore;
import java.security.PrivateKey;
import java.security.cert.Certificate;
import java.security.cert.CertificateFactory;
import java.security.cert.X509Certificate;
import java.security.spec.PKCS8EncodedKeySpec;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;

/**
 * Builds a server-side {@link SSLContext} from a PEM certificate chain and a PEM private key.
 *
 * <p>Pure Java 8 and free of {@code java.util.Base64} (missing on Android API 24/25), so the
 * same code runs on the Control Hub and in the desktop MockRobot. Supported key encodings:
 * PKCS#8 ({@code PRIVATE KEY}), PKCS#1 ({@code RSA PRIVATE KEY}) and SEC1
 * ({@code EC PRIVATE KEY}).</p>
 */
public final class PemTls {
    /** Result of {@link #load}. */
    public static final class Material {
        public final SSLContext context;
        /** First certificate of the chain, i.e. the one clients see as the server identity. */
        public final X509Certificate leaf;

        Material(SSLContext context, X509Certificate leaf) {
            this.context = context;
            this.leaf = leaf;
        }
    }

    private static final Pattern PEM_BLOCK = Pattern.compile(
            "-----BEGIN ([A-Z0-9 ]+)-----([^-]*)-----END \\1-----");

    private static final char[] KEY_PASSWORD = "azconductor".toCharArray();

    private static final int TAG_INTEGER = 0x02;
    private static final int TAG_OCTET_STRING = 0x04;
    private static final int TAG_NULL = 0x05;
    private static final int TAG_OID = 0x06;
    private static final int TAG_SEQUENCE = 0x30;
    private static final int TAG_CONTEXT_0 = 0xA0;

    private static final byte[] OID_RSA_ENCRYPTION = {
            0x06, 0x09, 0x2A, (byte) 0x86, 0x48, (byte) 0x86, (byte) 0xF7, 0x0D, 0x01, 0x01, 0x01};
    private static final byte[] OID_EC_PUBLIC_KEY = {
            0x06, 0x07, 0x2A, (byte) 0x86, 0x48, (byte) 0xCE, 0x3D, 0x02, 0x01};

    private PemTls() {}

    public static Material load(String fullchainPem, String privateKeyPem)
            throws GeneralSecurityException, IOException {
        List<Certificate> chain = parseCertificates(fullchainPem);
        if (chain.isEmpty()) throw new IOException("no CERTIFICATE block in chain");
        PrivateKey key = parsePrivateKey(privateKeyPem);

        KeyStore store = KeyStore.getInstance(KeyStore.getDefaultType());
        store.load(null, null);
        store.setKeyEntry("server", key, KEY_PASSWORD, chain.toArray(new Certificate[0]));

        KeyManagerFactory factory =
                KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());
        factory.init(store, KEY_PASSWORD);

        SSLContext context = SSLContext.getInstance("TLS");
        context.init(factory.getKeyManagers(), null, null);
        return new Material(context, (X509Certificate) chain.get(0));
    }

    static List<Certificate> parseCertificates(String pem) throws GeneralSecurityException, IOException {
        CertificateFactory factory = CertificateFactory.getInstance("X.509");
        List<Certificate> certificates = new ArrayList<>();
        Matcher matcher = PEM_BLOCK.matcher(pem);
        while (matcher.find()) {
            if (!"CERTIFICATE".equals(matcher.group(1))) continue;
            byte[] der = base64Decode(matcher.group(2));
            certificates.add(factory.generateCertificate(new ByteArrayInputStream(der)));
        }
        return certificates;
    }

    static PrivateKey parsePrivateKey(String pem) throws GeneralSecurityException, IOException {
        Matcher matcher = PEM_BLOCK.matcher(pem);
        while (matcher.find()) {
            String label = matcher.group(1);
            byte[] der = base64Decode(matcher.group(2));
            switch (label) {
                case "PRIVATE KEY":
                    return pkcs8(der);
                case "RSA PRIVATE KEY":
                    return pkcs8(wrapPkcs1(der));
                case "EC PRIVATE KEY":
                    return pkcs8(wrapSec1(der));
                case "ENCRYPTED PRIVATE KEY":
                    throw new IOException("encrypted private keys are not supported");
                default:
                    break;
            }
        }
        throw new IOException("no supported PRIVATE KEY block found");
    }

    private static PrivateKey pkcs8(byte[] der) throws GeneralSecurityException {
        PKCS8EncodedKeySpec spec = new PKCS8EncodedKeySpec(der);
        GeneralSecurityException last = null;
        for (String algorithm : new String[] {"RSA", "EC"}) {
            try {
                return KeyFactory.getInstance(algorithm).generatePrivate(spec);
            } catch (GeneralSecurityException e) {
                last = e;
            }
        }
        throw last;
    }

    // ---- DER helpers -------------------------------------------------------------------

    /** PKCS#1 RSAPrivateKey -> PKCS#8 PrivateKeyInfo. */
    private static byte[] wrapPkcs1(byte[] pkcs1) throws IOException {
        byte[] algorithm = tlv(TAG_SEQUENCE, OID_RSA_ENCRYPTION, new byte[] {TAG_NULL, 0});
        return tlv(TAG_SEQUENCE,
                new byte[] {TAG_INTEGER, 1, 0},
                algorithm,
                tlv(TAG_OCTET_STRING, pkcs1));
    }

    /** SEC1 ECPrivateKey -> PKCS#8 PrivateKeyInfo; the curve is taken from its [0] parameters. */
    private static byte[] wrapSec1(byte[] sec1) throws IOException {
        int[] top = readTlv(sec1, 0);
        if (top == null || sec1[0] != TAG_SEQUENCE) throw new IOException("malformed EC key");
        byte[] curve = null;
        int offset = top[1];
        while (offset < top[2]) {
            int[] item = readTlv(sec1, offset);
            if (item == null) throw new IOException("malformed EC key");
            if ((sec1[offset] & 0xFF) == TAG_CONTEXT_0) {
                int[] oid = readTlv(sec1, item[1]);
                if (oid != null && sec1[item[1]] == TAG_OID) {
                    // item[1] is where the nested OID's tag byte sits, so this keeps its header.
                    curve = Arrays.copyOfRange(sec1, item[1], oid[2]);
                }
            }
            offset = item[2];
        }
        if (curve == null) throw new IOException("EC key does not name its curve");

        byte[] algorithm = tlv(TAG_SEQUENCE, OID_EC_PUBLIC_KEY, curve);
        return tlv(TAG_SEQUENCE,
                new byte[] {TAG_INTEGER, 1, 0},
                algorithm,
                tlv(TAG_OCTET_STRING, sec1));
    }

    /** Returns {tag, valueStart, valueEnd} for the TLV at {@code offset}, or null if malformed. */
    private static int[] readTlv(byte[] data, int offset) {
        if (offset + 2 > data.length) return null;
        int tag = data[offset] & 0xFF;
        int first = data[offset + 1] & 0xFF;
        int start = offset + 2;
        int length;
        if (first < 0x80) {
            length = first;
        } else {
            int count = first & 0x7F;
            if (count == 0 || count > 3 || start + count > data.length) return null;
            length = 0;
            for (int i = 0; i < count; i++) length = (length << 8) | (data[start + i] & 0xFF);
            start += count;
        }
        int end = start + length;
        if (end > data.length || end < start) return null;
        return new int[] {tag, start, end};
    }

    private static byte[] tlv(int tag, byte[]... parts) {
        int length = 0;
        for (byte[] part : parts) length += part.length;
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        out.write(tag);
        if (length < 0x80) {
            out.write(length);
        } else if (length <= 0xFF) {
            out.write(0x81);
            out.write(length);
        } else if (length <= 0xFFFF) {
            out.write(0x82);
            out.write(length >> 8);
            out.write(length);
        } else {
            out.write(0x83);
            out.write(length >> 16);
            out.write(length >> 8);
            out.write(length);
        }
        for (byte[] part : parts) out.write(part, 0, part.length);
        return out.toByteArray();
    }

    // ---- Base64 (java.util.Base64 is unavailable below Android API 26) -----------------

    static byte[] base64Decode(String text) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        int buffer = 0;
        int bits = 0;
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if (c == '=' || Character.isWhitespace(c)) continue;
            int value;
            if (c >= 'A' && c <= 'Z') value = c - 'A';
            else if (c >= 'a' && c <= 'z') value = c - 'a' + 26;
            else if (c >= '0' && c <= '9') value = c - '0' + 52;
            else if (c == '+' || c == '-') value = 62;
            else if (c == '/' || c == '_') value = 63;
            else throw new IOException("invalid base64 character");
            buffer = (buffer << 6) | value;
            bits += 6;
            if (bits >= 8) {
                bits -= 8;
                out.write((buffer >> bits) & 0xFF);
            }
        }
        return out.toByteArray();
    }
}
