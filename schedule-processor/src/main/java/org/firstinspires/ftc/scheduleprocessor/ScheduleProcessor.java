package org.firstinspires.ftc.scheduleprocessor;

import java.io.IOException;
import java.io.Writer;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.type.TypeKind;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;

@SupportedAnnotationTypes("org.firstinspires.ftc.teamcode.common.subsystem.GenerateSchedule")
public final class ScheduleProcessor extends AbstractProcessor {
    private static final String ACTION = "org.firstinspires.ftc.teamcode.common.subsystem.ScheduledAction";
    private static final String DIRECT = "org.firstinspires.ftc.teamcode.common.subsystem.DirectAction";
    private static final String PARAM = "org.firstinspires.ftc.teamcode.common.util.Param";
    /** Mirrors {@code Param.NO_DEFAULT}; the processor cannot depend on TeamCode. */
    private static final String NO_DEFAULT = "\u0000";
    private static final Set<String> RESERVED = Set.of(
            "schedule", "execute", "cancel", "waitMillis", "waitUntil",
            "isCancelled", "isExecuted", "update", "shutdown", "cancelSchedule",
            "hasActiveSchedule", "periodic", "onScheduleCancelled", "onShutdown");

    @Override public SourceVersion getSupportedSourceVersion() {
        return SourceVersion.latestSupported();
    }

    @Override public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment round) {
        for (TypeElement annotation : annotations) {
            for (Element element : round.getElementsAnnotatedWith(annotation)) {
                if (element.getKind() != ElementKind.CLASS) {
                    error(element, "@GenerateSchedule requires a class");
                } else generate((TypeElement) element);
            }
        }
        return true;
    }

    private static boolean annotated(Element element, String name) {
        return element.getAnnotationMirrors().stream()
                .anyMatch(a -> a.getAnnotationType().toString().equals(name));
    }

    private void generate(TypeElement type) {
        String pkg = processingEnv.getElementUtils().getPackageOf(type).getQualifiedName().toString();
        String base = type.getSimpleName().toString();
        String scheduleName = base + "Schedule";
        String schedulerName = base + "Scheduler";
        StringBuilder schedule = new StringBuilder("package " + pkg + ";\n\npublic interface ")
                .append(scheduleName).append(" extends ScheduleApi<").append(scheduleName).append("> {\n");
        StringBuilder scheduler = new StringBuilder("package " + pkg + ";\n\npublic final class ")
                .append(schedulerName).append(" extends Scheduler<").append(scheduleName).append("> {\n")
                .append("    private final ").append(base).append(" target;\n")
                .append("    public ").append(schedulerName).append("(").append(base).append(" target) {\n")
                .append("        super(").append(scheduleName).append(".class, target);\n")
                .append("        this.target = target;\n    }\n");
        boolean valid = true;
        for (Element element : type.getEnclosedElements()) {
            if (element.getKind() != ElementKind.METHOD) continue;
            ExecutableElement method = (ExecutableElement) element;
            boolean markedSchedule = annotated(method, ACTION);
            boolean markedDirect = annotated(method, DIRECT);
            boolean publicInstance = method.getModifiers().contains(Modifier.PUBLIC)
                    && !method.getModifiers().contains(Modifier.STATIC);
            if (!publicInstance) {
                if (markedSchedule || markedDirect) {
                    error(method, "Annotated methods must be public instance methods");
                    valid = false;
                }
                continue;
            }
            String methodName = method.getSimpleName().toString();
            if (RESERVED.contains(methodName)) {
                if (markedSchedule || markedDirect) {
                    error(method, "Reserved lifecycle/scheduler method: " + methodName);
                    valid = false;
                }
                continue;
            }
            if (markedSchedule && markedDirect) {
                error(method, "Cannot combine @ScheduledAction with @DirectAction");
                valid = false;
                continue;
            }
            if (!method.getTypeParameters().isEmpty()
                    || !method.getThrownTypes().isEmpty()) {
                error(method, "Generated methods do not yet support generic method parameters or throws");
                valid = false;
                continue;
            }
            boolean delayed = markedSchedule || (!markedDirect && method.getReturnType().getKind() == TypeKind.VOID);
            StringBuilder signature = new StringBuilder(methodName).append("(");
            StringBuilder args = new StringBuilder();
            boolean first = true;
            int parameterIndex = 0;
            boolean sawDefault = false;
            for (VariableElement param : method.getParameters()) {
                if (!first) { signature.append(", "); args.append(", "); }
                first = false;
                String parameterType = param.asType().toString();
                if (method.isVarArgs() && parameterIndex == method.getParameters().size() - 1) {
                    parameterType = parameterType.substring(0, parameterType.length() - 2) + "...";
                }

                // Control Hub cannot read parameter names reflectively, so every generated
                // parameter carries @Param. A source-level @Param may rename the parameter
                // and give it a default value; both are copied through.
                String javaName = param.getSimpleName().toString();
                String paramName = javaName;
                String defaultValue = null;
                for (AnnotationMirror mirror : param.getAnnotationMirrors()) {
                    if (!mirror.getAnnotationType().toString().equals(PARAM)) continue;
                    for (var entry : mirror.getElementValues().entrySet()) {
                        String key = entry.getKey().getSimpleName().toString();
                        String text = String.valueOf(entry.getValue().getValue());
                        if (key.equals("value") && !text.isEmpty()) paramName = text;
                        if (key.equals("defaultValue") && !text.equals(NO_DEFAULT)) {
                            defaultValue = text;
                        }
                    }
                }
                if (defaultValue != null) {
                    sawDefault = true;
                } else if (sawDefault) {
                    error(param, "Parameters with a defaultValue must be the trailing "
                            + "parameters of the method; '" + javaName
                            + "' has no default but follows one that does");
                    valid = false;
                }

                signature.append("@").append(PARAM).append("(value = ")
                        .append(literal(paramName));
                if (defaultValue != null) {
                    signature.append(", defaultValue = ").append(literal(defaultValue));
                }
                signature.append(") ").append(parameterType).append(" ").append(javaName);
                args.append(javaName);
                parameterIndex++;
            }
            signature.append(")");
            if (delayed) {
                schedule.append("    ").append(scheduleName).append(" ").append(signature).append(";\n");
                // Shortcut on the scheduler: scheduler.m(args) == scheduler.schedule().m(args).execute().
                // It only publishes to the mailbox, so unlike direct methods it is safe from any thread.
                scheduler.append("    public ").append(scheduleName).append(" ").append(signature)
                        .append(" {\n        return schedule().").append(methodName)
                        .append("(").append(args).append(").execute();\n    }\n");
            } else {
                String returnType = method.getReturnType().toString();
                scheduler.append("    public ").append(returnType).append(" ").append(signature)
                        .append(" {\n        ");
                if (method.getReturnType().getKind() != TypeKind.VOID) scheduler.append("return ");
                scheduler.append("target.").append(methodName).append("(").append(args).append(");\n    }\n");
            }
        }
        if (!valid) return;
        schedule.append("}\n");
        scheduler.append("}\n");
        write(pkg + "." + scheduleName, schedule.toString(), type);
        write(pkg + "." + schedulerName, scheduler.toString(), type);
    }

    /** Java string literal that survives any characters, including the NO_DEFAULT sentinel. */
    private static String literal(String text) {
        StringBuilder out = new StringBuilder("\"");
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            switch (c) {
                case '"': out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\n': out.append("\\n"); break;
                case '\r': out.append("\\r"); break;
                case '\t': out.append("\\t"); break;
                default:
                    if (c < 0x20 || c > 0x7e) out.append(String.format("\\u%04x", (int) c));
                    else out.append(c);
            }
        }
        return out.append('"').toString();
    }

    private void write(String name, String content, TypeElement owner) {
        try {
            JavaFileObject file = processingEnv.getFiler().createSourceFile(name, owner);
            try (Writer writer = file.openWriter()) { writer.write(content); }
        } catch (IOException e) { error(owner, "Failed generating " + name + ": " + e.getMessage()); }
    }

    private void error(Element e, String message) {
        processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, message, e);
    }
}
