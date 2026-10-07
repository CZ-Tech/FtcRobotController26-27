package org.firstinspires.ftc.teamcode.common.network;

import java.util.List;

/**
 * Storage abstraction for Network V2 routes.
 *
 * <p>The protocol/router depends only on this contract so the Android robot controller
 * can use SharedPreferences while the desktop MockRobot can use an in-memory/file-backed
 * implementation without duplicating the network stack.</p>
 */
public interface RouteRepository {
    final class Entry {
        public final String name;
        public final String json;
        public final long revision;

        public Entry(String name, String json, long revision) {
            this.name = name;
            this.json = json;
            this.revision = revision;
        }
    }

    final class PutResult {
        public final boolean updated;
        public final boolean preconditionFailed;
        public final Entry entry;

        public PutResult(boolean updated, boolean preconditionFailed, Entry entry) {
            this.updated = updated;
            this.preconditionFailed = preconditionFailed;
            this.entry = entry;
        }
    }

    List<Entry> list();

    Entry get(String name);

    PutResult put(String name, String json, Long expectedRevision);

    boolean delete(String name, Long expectedRevision);

    long changeRevision();

    String manifestJson();
}
