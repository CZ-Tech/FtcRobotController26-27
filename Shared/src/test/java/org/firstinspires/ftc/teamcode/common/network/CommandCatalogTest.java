package org.firstinspires.ftc.teamcode.common.network;

import org.firstinspires.ftc.teamcode.common.network.CommandCatalog.Descriptor;
import org.firstinspires.ftc.teamcode.common.network.CommandCatalog.Signature;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class CommandCatalogTest {
    private static Signature signature(List<String> names, List<String> types) {
        return new Signature(names, types, null);
    }

    private static void assertSeparateSignatures(Descriptor descriptor, Integer... counts) {
        assertEquals(Arrays.asList(counts), descriptor.acceptedArgCounts);
        assertFalse(descriptor.hasUnifiedSignature);
        assertNull(descriptor.requiredParamCount);
        assertNull(descriptor.parameterRequired);
    }

    private static void assertUnmodifiable(Runnable mutation) {
        try {
            mutation.run();
            fail("Metadata must be immutable");
        } catch (UnsupportedOperationException expected) { }
    }

    private static void assertSignatureJson(JSONObject json, String names,
                                            String types, String defaults) {
        assertEquals(names, json.getJSONArray("paramNames").toString());
        assertEquals(types, json.getJSONArray("paramTypes").toString());
        assertEquals(defaults, json.getJSONArray("paramDefaults").toString());
    }

    @Test
    public void zeroAndOneArgumentOverloadsAreOptionalWithoutAnInferredDefault() {
        // The legacy constructor synthesizes the shorter overload's name prefix.
        Descriptor descriptor = new Descriptor("spin",
                Collections.singletonList("power"), Collections.singletonList("double"),
                Arrays.asList(Collections.<String>emptyList(),
                        Collections.singletonList("double")));

        assertEquals(Arrays.asList(0, 1), descriptor.acceptedArgCounts);
        assertTrue(descriptor.hasUnifiedSignature);
        assertEquals(Integer.valueOf(0), descriptor.requiredParamCount);
        assertEquals(Collections.singletonList(false), descriptor.parameterRequired);
        assertEquals(Collections.singletonList(null), descriptor.parameterDefaults);
        assertEquals(Collections.emptyList(), descriptor.signatures.get(0).parameterNames);
        assertEquals(Collections.emptyList(), descriptor.signatures.get(0).parameterDefaults);
        assertEquals(Collections.singletonList("power"),
                descriptor.signatures.get(1).parameterNames);
        assertEquals(Collections.emptyList(), descriptor.typesForArity(0));
        assertEquals(Collections.singletonList("double"), descriptor.typesForArity(1));
        assertNull(descriptor.typesForArity(-1));
        assertNull(descriptor.typesForArity(2));
    }

    @Test
    public void fixedSignatureRequiresEveryParameterEvenWhenDefaultsAreUnknown() {
        Descriptor descriptor = new Descriptor("shoot", Arrays.asList("count", "power"),
                Arrays.asList("int", "double"));

        assertEquals(Collections.singletonList(2), descriptor.acceptedArgCounts);
        assertTrue(descriptor.hasUnifiedSignature);
        assertEquals(Integer.valueOf(2), descriptor.requiredParamCount);
        assertEquals(Arrays.asList(true, true), descriptor.parameterRequired);
        assertEquals(Arrays.asList(null, null), descriptor.parameterDefaults);
        assertNull(descriptor.typesForArity(1));
        assertEquals(Arrays.asList("int", "double"), descriptor.typesForArity(2));
    }

    @Test
    public void explicitAnnotationDefaultsArePreservedSeparatelyFromRequiredFlags() {
        Descriptor descriptor = new Descriptor("shoot", Arrays.asList("count", "power"),
                Arrays.asList("int", "double"), Arrays.asList(null, "0.8"),
                Arrays.asList(Collections.singletonList("int"), Arrays.asList("int", "double")));

        assertEquals(Arrays.asList(1, 2), descriptor.acceptedArgCounts);
        assertTrue(descriptor.hasUnifiedSignature);
        assertEquals(Integer.valueOf(1), descriptor.requiredParamCount);
        assertEquals(Arrays.asList(true, false), descriptor.parameterRequired);
        assertEquals(Arrays.asList(null, "0.8"), descriptor.parameterDefaults);
        assertEquals(Collections.singletonList("count"),
                descriptor.signatures.get(0).parameterNames);
        assertEquals(Collections.singletonList(null),
                descriptor.signatures.get(0).parameterDefaults);
        assertEquals(Arrays.asList(null, "0.8"),
                descriptor.signatures.get(1).parameterDefaults);

        // A default annotation alone does not add an accepted invocation.
        Descriptor fixed = Descriptor.fromSignatures("fixed", Collections.singletonList(
                new Signature(Collections.singletonList("power"),
                        Collections.singletonList("double"), Collections.singletonList("0.8"))));
        assertEquals(Integer.valueOf(1), fixed.requiredParamCount);
        assertEquals(Collections.singletonList(true), fixed.parameterRequired);
        assertNull(fixed.typesForArity(0));
    }

    @Test
    public void arityGapCannotBePresentedAsOneOptionalTail() {
        Descriptor descriptor = Descriptor.fromSignatures("gap", Arrays.asList(
                signature(Collections.<String>emptyList(), Collections.<String>emptyList()),
                signature(Arrays.asList("count", "power"), Arrays.asList("int", "double"))));

        assertSeparateSignatures(descriptor, 0, 2);
        assertEquals(Collections.emptyList(), descriptor.typesForArity(0));
        assertNull(descriptor.typesForArity(1));
        assertEquals(Arrays.asList("int", "double"), descriptor.typesForArity(2));
    }

    @Test
    public void differentTypePrefixesRemainSeparateAndKeepTheirExactInvocationTypes() {
        Descriptor descriptor = Descriptor.fromSignatures("change", Arrays.asList(
                signature(Collections.singletonList("value"), Collections.singletonList("boolean")),
                signature(Arrays.asList("value", "power"), Arrays.asList("int", "double"))));

        assertSeparateSignatures(descriptor, 1, 2);
        assertEquals(Collections.singletonList("boolean"), descriptor.typesForArity(1));
        assertEquals(Arrays.asList("int", "double"), descriptor.typesForArity(2));
    }

    @Test
    public void differentNamePrefixesRemainSeparateEvenWhenTypesMatch() {
        Descriptor descriptor = Descriptor.fromSignatures("change", Arrays.asList(
                signature(Collections.singletonList("duration"), Collections.singletonList("double")),
                signature(Arrays.asList("power", "enabled"), Arrays.asList("double", "boolean"))));

        assertSeparateSignatures(descriptor, 1, 2);
        assertEquals(Collections.singletonList("duration"),
                descriptor.signatures.get(0).parameterNames);
        assertEquals(Arrays.asList("power", "enabled"), descriptor.parameterNames);
        assertEquals(Collections.singletonList("double"), descriptor.typesForArity(1));
    }

    @Test
    public void signaturesAreSortedByArityRegardlessOfInputOrder() {
        Signature zero = signature(Collections.<String>emptyList(), Collections.<String>emptyList());
        Signature one = signature(Collections.singletonList("count"), Collections.singletonList("int"));
        Signature two = new Signature(Arrays.asList("count", "power"),
                Arrays.asList("int", "double"), Arrays.asList(null, "0.8"));
        Descriptor first = Descriptor.fromSignatures("shoot", Arrays.asList(two, zero, one));
        Descriptor second = Descriptor.fromSignatures("shoot", Arrays.asList(one, two, zero));

        assertEquals(Arrays.asList(zero, one, two), first.signatures);
        assertEquals(first.signatures, second.signatures);
        assertEquals(Arrays.asList(0, 1, 2), first.acceptedArgCounts);
        assertTrue(first.hasUnifiedSignature);
        assertEquals(Integer.valueOf(0), first.requiredParamCount);
        assertEquals(Arrays.asList(false, false), first.parameterRequired);
        assertEquals(two.parameterNames, first.parameterNames);
        assertEquals(two.parameterTypes, first.parameterTypes);
        assertEquals(two.parameterDefaults, first.parameterDefaults);
    }

    @Test
    public void signatureCopiesInputsAndExposesOnlyImmutableLists() {
        List<String> names = new ArrayList<>(Arrays.asList("count", "power"));
        List<String> types = new ArrayList<>(Arrays.asList("int", "double"));
        List<String> defaults = new ArrayList<>(Collections.singletonList("3"));
        Signature signature = new Signature(names, types, defaults);
        names.set(0, "changed");
        types.clear();
        defaults.set(0, "99");

        assertEquals(Arrays.asList("count", "power"), signature.parameterNames);
        assertEquals(Arrays.asList("int", "double"), signature.parameterTypes);
        assertEquals(Arrays.asList("3", null), signature.parameterDefaults);
        assertUnmodifiable(() -> signature.parameterNames.set(0, "changed"));
        assertUnmodifiable(() -> signature.parameterTypes.add("boolean"));
        assertUnmodifiable(() -> signature.parameterDefaults.set(0, "99"));
    }

    @Test
    public void descriptorCopiesSignatureListAndExposesImmutableMetadata() {
        List<Signature> source = new ArrayList<>(Arrays.asList(
                signature(Collections.singletonList("power"), Collections.singletonList("double")),
                signature(Collections.<String>emptyList(), Collections.<String>emptyList())));
        Descriptor descriptor = Descriptor.fromSignatures("spin", source);
        source.clear();

        assertEquals(2, descriptor.signatures.size());
        assertEquals(Arrays.asList(0, 1), descriptor.acceptedArgCounts);
        assertUnmodifiable(() -> descriptor.signatures.clear());
        assertUnmodifiable(() -> descriptor.acceptedArgCounts.add(2));
        assertUnmodifiable(() -> descriptor.parameterRequired.set(0, true));
        assertUnmodifiable(() -> descriptor.parameterNames.set(0, "changed"));
        assertUnmodifiable(() -> descriptor.parameterTypes.set(0, "int"));
        assertUnmodifiable(() -> descriptor.parameterDefaults.set(0, "0.8"));
        assertUnmodifiable(() -> descriptor.typesForArity(1).set(0, "int"));
    }

    @Test
    public void legacyConstructorCopiesNamesDefaultsAndNestedAcceptedTypes() {
        List<String> names = new ArrayList<>(Arrays.asList("count", "power"));
        List<String> types = new ArrayList<>(Arrays.asList("int", "double"));
        List<String> defaults = new ArrayList<>(Arrays.asList(null, "0.8"));
        List<String> shorter = new ArrayList<>(Collections.singletonList("int"));
        List<List<String>> accepted = new ArrayList<>(Arrays.asList(types, shorter));
        Descriptor descriptor = new Descriptor("shoot", names, types, defaults, accepted);
        names.clear();
        types.set(0, "boolean");
        defaults.set(1, "1.0");
        shorter.clear();
        accepted.clear();

        assertEquals(Arrays.asList("count", "power"), descriptor.parameterNames);
        assertEquals(Arrays.asList("int", "double"), descriptor.parameterTypes);
        assertEquals(Arrays.asList(null, "0.8"), descriptor.parameterDefaults);
        assertEquals(Arrays.asList(1, 2), descriptor.acceptedArgCounts);
        assertEquals(Collections.singletonList("count"),
                descriptor.signatures.get(0).parameterNames);
        assertEquals(Collections.singletonList("int"), descriptor.typesForArity(1));
    }

    @Test
    public void jsonIncludesUnifiedRequiredFlagsWithoutInventingDefaults() {
        CommandCatalog catalog = new CommandCatalog();
        catalog.publish(Collections.singletonList(Descriptor.fromSignatures("spin", Arrays.asList(
                signature(Collections.singletonList("power"), Collections.singletonList("double")),
                signature(Collections.<String>emptyList(), Collections.<String>emptyList())))));
        JSONObject root = new JSONObject(catalog.toJson());
        JSONObject command = root.getJSONArray("commands").getJSONObject(0);

        assertEquals(catalog.revision(), root.getLong("revision"));
        assertEquals("spin", command.getString("name"));
        assertSignatureJson(command, "[\"power\"]", "[\"double\"]", "[null]");
        assertEquals("[0,1]", command.getJSONArray("acceptedArgCounts").toString());
        assertTrue(command.getBoolean("hasUnifiedSignature"));
        assertEquals(0, command.getInt("requiredParamCount"));
        assertEquals("[false]", command.getJSONArray("paramRequired").toString());
        JSONArray signatures = command.getJSONArray("signatures");
        assertEquals(2, signatures.length());
        assertSignatureJson(signatures.getJSONObject(0), "[]", "[]", "[]");
        assertSignatureJson(signatures.getJSONObject(1), "[\"power\"]", "[\"double\"]", "[null]");
        assertTrue(signatures.getJSONObject(1).getJSONArray("paramDefaults").isNull(0));
    }

    @Test
    public void jsonIncludesExplicitNullsAndCompleteNonUnifiedSignatures() {
        CommandCatalog catalog = new CommandCatalog();
        catalog.publish(Collections.singletonList(Descriptor.fromSignatures("change", Arrays.asList(
                new Signature(Arrays.asList("count", "power"), Arrays.asList("int", "double"),
                        Arrays.asList(null, "0.8")),
                new Signature(Collections.singletonList("enabled"), Collections.singletonList("boolean"),
                        Collections.singletonList("true"))))));
        JSONObject command = new JSONObject(catalog.toJson()).getJSONArray("commands").getJSONObject(0);

        assertEquals("change", command.getString("name"));
        assertSignatureJson(command, "[\"count\",\"power\"]", "[\"int\",\"double\"]", "[null,\"0.8\"]");
        assertEquals("[1,2]", command.getJSONArray("acceptedArgCounts").toString());
        assertFalse(command.getBoolean("hasUnifiedSignature"));
        assertTrue(command.has("requiredParamCount"));
        assertTrue(command.isNull("requiredParamCount"));
        assertTrue(command.has("paramRequired"));
        assertTrue(command.isNull("paramRequired"));
        JSONArray signatures = command.getJSONArray("signatures");
        assertEquals(2, signatures.length());
        assertSignatureJson(signatures.getJSONObject(0), "[\"enabled\"]", "[\"boolean\"]", "[\"true\"]");
        assertSignatureJson(signatures.getJSONObject(1), "[\"count\",\"power\"]",
                "[\"int\",\"double\"]", "[null,\"0.8\"]");
    }

    @Test(expected = IllegalArgumentException.class)
    public void duplicateAritiesAreRejectedEvenWhenTypesDiffer() {
        Descriptor.fromSignatures("ambiguous", Arrays.asList(
                signature(Collections.singletonList("enabled"), Collections.singletonList("boolean")),
                signature(Collections.singletonList("count"), Collections.singletonList("int"))));
    }

    @Test(expected = IllegalArgumentException.class)
    public void emptySignaturesAreRejected() {
        Descriptor.fromSignatures("missing", Collections.<Signature>emptyList());
    }
}
