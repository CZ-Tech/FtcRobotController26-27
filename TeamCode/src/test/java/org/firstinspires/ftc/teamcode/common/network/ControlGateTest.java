package org.firstinspires.ftc.teamcode.common.network;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Arrays;

public class ControlGateTest {

    @Test
    public void inactiveOperationsAreDroppedAndNeverStored() {
        ControlMailbox mailbox = new ControlMailbox();
        ControlGate gate = new ControlGate(mailbox);

        assertNull(gate.submitSavedPath("auto"));
        assertNull(gate.submitInlinePath("[]"));
        assertNull(gate.submitCommand("shoot", Arrays.<Object>asList(1)));
        assertNull(gate.pollLatest());
        assertFalse(mailbox.hasPending());
    }

    @Test
    public void latestOperationReplacesOlderUnconsumedOperation() {
        ControlMailbox mailbox = new ControlMailbox();
        ControlGate gate = new ControlGate(mailbox);
        gate.activate();

        ControlRequest first = gate.submitSavedPath("first");
        ControlRequest second = gate.submitInlinePath("[{\"x\":1}]");
        ControlRequest third = gate.submitCommand("latest", Arrays.<Object>asList("x"));

        assertTrue(first.id < second.id);
        assertTrue(second.id < third.id);

        ControlRequest consumed = gate.pollLatest();
        assertSame(third, consumed);
        assertEquals(ControlRequest.Type.RUN_COMMAND, consumed.type);
        assertEquals("latest", consumed.commandName);
        assertNull(gate.pollLatest());
    }

    @Test
    public void deactivateAtomicallyClearsPendingOperation() {
        ControlMailbox mailbox = new ControlMailbox();
        ControlGate gate = new ControlGate(mailbox);
        gate.activate();
        gate.submitSavedPath("stale");

        gate.deactivate();

        assertFalse(gate.isActive());
        assertFalse(mailbox.hasPending());
        assertNull(gate.pollLatest());
    }

    @Test
    public void newActivationCannotConsumePreviousGenerationOperation() {
        ControlMailbox mailbox = new ControlMailbox();
        ControlGate gate = new ControlGate(mailbox);

        long firstGeneration = gate.activate();
        gate.submitSavedPath("old-opmode");
        gate.deactivate();
        long secondGeneration = gate.activate();

        assertTrue(secondGeneration > firstGeneration);
        assertNull(gate.pollLatest());

        ControlRequest fresh = gate.submitSavedPath("new-opmode");
        assertSame(fresh, gate.pollLatest());
    }
}
