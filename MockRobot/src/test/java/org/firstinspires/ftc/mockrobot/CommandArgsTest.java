package org.firstinspires.ftc.mockrobot;

import org.firstinspires.ftc.teamcode.common.network.CommandArgs;
import org.junit.Test;

import java.util.Arrays;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class CommandArgsTest {
    @Test
    public void convertsEveryRecognizedTypeAndWrapper() {
        assertEquals(3, CommandArgs.convert("3", "int"));
        assertEquals(3, CommandArgs.convert("3", "java.lang.Integer"));
        assertEquals(3L, CommandArgs.convert("3", "long"));
        assertEquals((short) 3, CommandArgs.convert("3", "short"));
        assertEquals((byte) 3, CommandArgs.convert("3", "Byte"));
        assertEquals(0.5f, CommandArgs.convert("0.5", "float"));
        assertEquals(0.8, CommandArgs.convert("0.8", "double"));
        assertEquals(true, CommandArgs.convert("TRUE", "boolean"));
        assertEquals(false, CommandArgs.convert("false", "java.lang.Boolean"));
        assertEquals('x', CommandArgs.convert("x", "char"));
        assertEquals('x', CommandArgs.convert("x", "Character"));
    }

    @Test
    public void unknownTypesAreText() {
        assertEquals("anything", CommandArgs.convert("anything", "java.lang.String"));
        assertEquals("3", CommandArgs.convert("3", "com.example.Mode"));
    }

    @Test
    public void rejectsBadValues() {
        assertInvalid("abc", "int");
        assertInvalid("3.0", "int");
        assertInvalid("", "double");
        assertInvalid("NaN", "double");
        assertInvalid("Infinity", "float");
        assertInvalid("1e999", "double");
        assertInvalid("yes", "boolean");
        assertInvalid("", "boolean");
        assertInvalid("", "char");
        assertInvalid("ab", "char");
        assertInvalid("300", "byte");
    }

    @Test
    public void convertAllChecksCountAndReportsArgumentIndex() {
        assertArrayEquals(new Object[]{3, 0.8},
                CommandArgs.convertAll(Arrays.asList("int", "double"), Arrays.asList("3", "0.8")));

        try {
            CommandArgs.convertAll(Arrays.asList("int", "double"), Arrays.asList("3"));
            fail("count mismatch should fail");
        } catch (IllegalArgumentException expected) {
            // expected
        }

        try {
            CommandArgs.convertAll(Arrays.asList("int", "double"), Arrays.asList("3", "x"));
            fail("bad value should fail");
        } catch (IllegalArgumentException expected) {
            assertEquals(true, expected.getMessage().startsWith("Argument 1:"));
        }
    }

    private static void assertInvalid(String value, String type) {
        try {
            CommandArgs.convert(value, type);
            fail("'" + value + "' should not be a valid " + type);
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }
}
