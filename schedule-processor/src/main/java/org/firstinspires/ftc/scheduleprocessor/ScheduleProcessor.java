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
            if (!method.getTypeParameters().isEmpty() || method.isVarArgs()
                    || !method.getThrownTypes().isEmpty()) {
                error(method, "Generated methods do not yet support generic method parameters, varargs or throws");
                valid = false;
                continue;
            }
            boolean delayed = markedSchedule || (!markedDirect && method.getReturnType().getKind() == TypeKind.VOID);
            StringBuilder signature = new StringBuilder(methodName).append("(");
            StringBuilder args = new StringBuilder();
            boolean first = true;
            for (VariableElement param : method.getParameters()) {
                if (!first) { signature.append(", "); args.append(", "); }
                first = false;
                signature.append(param.asType()).append(" ").append(param.getSimpleName());
                args.append(param.getSimpleName());
            }
            signature.append(")");
            if (delayed) {
                schedule.append("    ").append(scheduleName).append(" ").append(signature).append(";\n");
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
