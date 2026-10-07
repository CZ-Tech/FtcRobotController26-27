package org.firstinspires.ftc.mockrobot

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.Checkbox
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExposedDropdownMenuBox
import androidx.compose.material3.ExposedDropdownMenuDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.application
import androidx.compose.ui.window.rememberWindowState
import androidx.compose.ui.unit.DpSize
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.common.network.ExecutionStateStore
import org.firstinspires.ftc.teamcode.common.network.RouteRepository
import org.firstinspires.ftc.teamcode.common.opmode.OpModeLifecycleService
import java.awt.Desktop
import java.nio.file.Files
import java.nio.file.Path as NioPath
import java.util.Locale
import java.util.concurrent.CountDownLatch
import java.util.concurrent.ConcurrentLinkedQueue
import kotlin.math.hypot
import kotlin.math.min

private const val FIELD_SIZE_IN = 144.0

fun main(args: Array<String>) {
    val robot = MockRobotRuntime()
    try {
        robot.start()
    } catch (error: Exception) {
        System.err.println("MockRobot failed to bind :8888: $error")
        robot.close()
        return
    }

    if ("--headless" in args) {
        MockLog.info("MockRobot", "Headless mode active on :8888")
        Runtime.getRuntime().addShutdownHook(Thread { robot.close() })
        try {
            CountDownLatch(1).await()
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        return
    }

    application {
        val windowState = rememberWindowState(size = DpSize(1320.dp, 860.dp))
        Window(
            onCloseRequest = {
                robot.close()
                exitApplication()
            },
            title = "FTC MockRobot — V2 :8888",
            state = windowState,
        ) {
            MaterialTheme(colorScheme = darkColorScheme()) {
                MockRobotApp(robot)
            }
        }
    }
}

private data class UiSnapshot(
    val opMode: OpModeLifecycleService.Snapshot,
    val simulation: MockSimulationEngine.Snapshot,
    val execution: ExecutionStateStore.Snapshot,
    val sessionOwned: Boolean,
    val sessionOwner: String?,
    val sessionRemainingMs: Long,
    val routes: List<RouteRepository.Entry>,
)

@Composable
private fun MockRobotApp(robot: MockRobotRuntime) {
    var snapshot by remember {
        mutableStateOf(
            UiSnapshot(
                opMode = robot.opModes.snapshot(),
                simulation = robot.simulation.snapshot(),
                execution = robot.execution.snapshot(),
                sessionOwned = robot.session.isOwned,
                sessionOwner = robot.session.owner(),
                sessionRemainingMs = robot.session.expiresInMs(),
                routes = robot.routes.list(),
            )
        )
    }
    val logs = remember { mutableStateListOf<String>() }
    val pendingLogs = remember { ConcurrentLinkedQueue<String>() }

    DisposableEffect(robot) {
        val listener = java.util.function.Consumer<String> { line ->
            pendingLogs.add(line)
        }
        MockLog.addListener(listener)
        onDispose { MockLog.removeListener(listener) }
    }

    LaunchedEffect(robot) {
        while (true) {
            while (true) {
                val line = pendingLogs.poll() ?: break
                logs += line
            }
            while (logs.size > 800) logs.removeAt(0)
            snapshot = UiSnapshot(
                opMode = robot.opModes.snapshot(),
                simulation = robot.simulation.snapshot(),
                execution = robot.execution.snapshot(),
                sessionOwned = robot.session.isOwned,
                sessionOwner = robot.session.owner(),
                sessionRemainingMs = robot.session.expiresInMs(),
                routes = robot.routes.list(),
            )
            delay(50)
        }
    }

    Scaffold(
        topBar = { StatusBar(robot, snapshot) },
    ) { padding ->
        BoxWithConstraints(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(8.dp)
        ) {
            val compact = maxWidth < 1000.dp
            if (compact) {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .verticalScroll(rememberScrollState()),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    FieldCard(
                        robot = robot,
                        snapshot = snapshot,
                        modifier = Modifier
                            .fillMaxWidth()
                            .heightIn(min = 420.dp, max = 700.dp),
                    )
                    ControlPanel(
                        robot,
                        snapshot,
                        Modifier.fillMaxWidth().height(620.dp),
                    )
                    LogPanel(logs, Modifier.fillMaxWidth().height(180.dp))
                }
            } else {
                Column(
                    modifier = Modifier.fillMaxSize(),
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Row(
                        modifier = Modifier.weight(1f).fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        FieldCard(
                            robot = robot,
                            snapshot = snapshot,
                            modifier = Modifier.weight(1f).fillMaxHeight(),
                        )
                        ControlPanel(
                            robot,
                            snapshot,
                            Modifier.widthIn(min = 360.dp, max = 470.dp).fillMaxHeight(),
                        )
                    }
                    LogPanel(logs, Modifier.fillMaxWidth().height(170.dp))
                }
            }
        }
    }
}

@Composable
private fun StatusBar(robot: MockRobotRuntime, snapshot: UiSnapshot) {
    Surface(tonalElevation = 3.dp) {
        FlowRow(
            modifier = Modifier.fillMaxWidth().padding(horizontal = 10.dp, vertical = 6.dp),
            horizontalArrangement = Arrangement.spacedBy(16.dp),
            verticalArrangement = Arrangement.spacedBy(4.dp),
            itemVerticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                if (robot.isRunning) "● :8888" else "○ SERVER",
                color = if (robot.isRunning) Color(0xFF68D391) else Color(0xFFFF6B6B),
                fontWeight = FontWeight.Bold,
            )
            Text(
                if (snapshot.sessionOwned) {
                    "Session: ${snapshot.sessionOwner ?: "unknown"} · ${snapshot.sessionRemainingMs}ms"
                } else {
                    "Session: —"
                }
            )
            Text(
                "OpMode: ${snapshot.opMode.phase}"
                    + (snapshot.opMode.activeName?.let { " · $it" } ?: "")
            )
            Text(
                String.format(
                    Locale.US,
                    "X %.1f · Y %.1f · H %.1f°",
                    snapshot.simulation.x,
                    snapshot.simulation.y,
                    snapshot.simulation.heading,
                )
            )
            Text(
                String.format(Locale.US, "误差 %.2f in", snapshot.simulation.trackingError),
                color = if (snapshot.simulation.trackingError < 2.0) {
                    Color(0xFF68D391)
                } else {
                    Color(0xFFFFC857)
                },
            )
        }
    }
}

@Composable
private fun FieldCard(
    robot: MockRobotRuntime,
    snapshot: UiSnapshot,
    modifier: Modifier = Modifier,
) {
    Card(modifier = modifier, shape = RoundedCornerShape(12.dp)) {
        BoxWithConstraints(
            modifier = Modifier.fillMaxSize().padding(8.dp),
            contentAlignment = Alignment.Center,
        ) {
            val fieldDp = if (maxWidth < maxHeight) maxWidth else maxHeight
            Box(
                modifier = Modifier
                    .size(fieldDp)
                    .background(Color(0xFF222222))
                    .pointerInput(snapshot.simulation.x, snapshot.simulation.y) {
                        detectDragGestures(
                            onDragStart = { start ->
                                val robotPx = logicalToScreen(
                                    snapshot.simulation.x,
                                    snapshot.simulation.y,
                                    size.width.toFloat(),
                                    size.height.toFloat(),
                                )
                                if (hypot(
                                        (start.x - robotPx.x).toDouble(),
                                        (start.y - robotPx.y).toDouble(),
                                    ) < 50.0
                                ) {
                                    robot.simulation.beginDrag()
                                }
                            },
                            onDragEnd = { robot.simulation.endDrag() },
                            onDragCancel = { robot.simulation.endDrag() },
                            onDrag = { change, _ ->
                                val logical = screenToLogical(
                                    change.position.x,
                                    change.position.y,
                                    size.width.toFloat(),
                                    size.height.toFloat(),
                                )
                                robot.simulation.dragTo(logical.x.toDouble(), logical.y.toDouble())
                                change.consume()
                            },
                        )
                    }
            ) {
                Image(
                    painter = painterResource("field/FTC_MAP26.png"),
                    contentDescription = "FTC field",
                    modifier = Modifier.fillMaxSize(),
                    contentScale = ContentScale.Fit,
                )
                Canvas(Modifier.fillMaxSize()) {
                    val plan = robot.simulation.activePlan() ?: selectedPreviewPlan(robot, snapshot)
                    if (plan != null) {
                        val points = plan.polyline(36)
                        if (points.size >= 2) {
                            val path = Path()
                            val first = logicalToScreen(
                                points.first().x,
                                points.first().y,
                                size.width,
                                size.height,
                            )
                            path.moveTo(first.x, first.y)
                            points.drop(1).forEach { point ->
                                val p = logicalToScreen(
                                    point.x,
                                    point.y,
                                    size.width,
                                    size.height,
                                )
                                path.lineTo(p.x, p.y)
                            }
                            drawPath(
                                path = path,
                                color = Color(0xFF4FC3F7),
                                style = Stroke(width = 3f, cap = StrokeCap.Round),
                            )
                        }
                    }

                    snapshot.simulation.idealX?.let { idealX ->
                        val idealY = snapshot.simulation.idealY ?: return@let
                        val ideal = logicalToScreen(idealX, idealY, size.width, size.height)
                        drawCircle(
                            color = Color.White.copy(alpha = 0.55f),
                            radius = 8f,
                            center = ideal,
                            style = Stroke(width = 2f),
                        )
                    }

                    val robotCenter = logicalToScreen(
                        snapshot.simulation.x,
                        snapshot.simulation.y,
                        size.width,
                        size.height,
                    )
                    drawCircle(
                        color = Color(0xFFFF5C48),
                        radius = 13f,
                        center = robotCenter,
                    )
                    val headingRadians = Math.toRadians(snapshot.simulation.heading + 90.0)
                    drawLine(
                        color = Color.White,
                        start = robotCenter,
                        end = Offset(
                            robotCenter.x + kotlin.math.cos(headingRadians).toFloat() * 22f,
                            robotCenter.y + kotlin.math.sin(headingRadians).toFloat() * 22f,
                        ),
                        strokeWidth = 3f,
                        cap = StrokeCap.Round,
                    )
                }

                Surface(
                    modifier = Modifier.align(Alignment.BottomStart).padding(8.dp),
                    color = Color.Black.copy(alpha = 0.65f),
                    shape = RoundedCornerShape(8.dp),
                ) {
                    Text(
                        "拖动红点改变 X/Y；右侧可精确修改 heading",
                        modifier = Modifier.padding(horizontal = 8.dp, vertical = 5.dp),
                        fontSize = 11.sp,
                    )
                }
            }
        }
    }
}

private fun logicalToScreen(x: Double, y: Double, width: Float, height: Float): Offset {
    val scale = min(width, height) / FIELD_SIZE_IN.toFloat()
    return Offset(
        x = width / 2f + y.toFloat() * scale,
        y = height / 2f + x.toFloat() * scale,
    )
}

private fun screenToLogical(px: Float, py: Float, width: Float, height: Float): Offset {
    val scale = min(width, height) / FIELD_SIZE_IN.toFloat()
    return Offset(
        x = (py - height / 2f) / scale,
        y = (px - width / 2f) / scale,
    )
}

@Composable
private fun ControlPanel(
    robot: MockRobotRuntime,
    snapshot: UiSnapshot,
    modifier: Modifier = Modifier,
) {
    var tab by remember { mutableStateOf(0) }
    Card(modifier = modifier, shape = RoundedCornerShape(12.dp)) {
        Column(Modifier.fillMaxSize()) {
            TabRow(selectedTabIndex = tab) {
                listOf("运行", "仿真", "网络/数据").forEachIndexed { index, title ->
                    Tab(
                        selected = tab == index,
                        onClick = { tab = index },
                        text = { Text(title) },
                    )
                }
            }
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .verticalScroll(rememberScrollState())
                    .padding(12.dp)
            ) {
                when (tab) {
                    0 -> RunControls(robot, snapshot)
                    1 -> SimulationControls(robot, snapshot)
                    else -> NetworkControls(robot, snapshot)
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun RunControls(robot: MockRobotRuntime, snapshot: UiSnapshot) {
    var selectedName by remember { mutableStateOf(robot.opModeBackend.profiles().first().name) }
    var opModeExpanded by remember { mutableStateOf(false) }
    var routeExpanded by remember { mutableStateOf(false) }

    val profile = robot.opModeBackend.profile(selectedName)
        ?: robot.opModeBackend.profiles().first()
    val routeNames = snapshot.routes.map { it.name }
    var routeName by remember(selectedName, snapshot.routes) {
        mutableStateOf(profile.routeName() ?: "")
    }

    Column(verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Text("Autonomous", style = MaterialTheme.typography.titleMedium)

        ExposedDropdownMenuBox(
            expanded = opModeExpanded,
            onExpandedChange = { opModeExpanded = it },
        ) {
            OutlinedTextField(
                value = selectedName,
                onValueChange = {},
                readOnly = true,
                trailingIcon = {
                    ExposedDropdownMenuDefaults.TrailingIcon(expanded = opModeExpanded)
                },
                modifier = Modifier.fillMaxWidth()
                    .menuAnchor(),
            )
            ExposedDropdownMenu(
                expanded = opModeExpanded,
                onDismissRequest = { opModeExpanded = false },
            ) {
                robot.opModeBackend.profiles().forEach { item ->
                    DropdownMenuItem(
                        text = { Text(item.name) },
                        onClick = {
                            selectedName = item.name
                            routeName = item.routeName() ?: ""
                            opModeExpanded = false
                        },
                    )
                }
            }
        }

        ExposedDropdownMenuBox(
            expanded = routeExpanded,
            onExpandedChange = { routeExpanded = it },
        ) {
            OutlinedTextField(
                value = routeName.ifBlank { "无路径" },
                onValueChange = {},
                readOnly = true,
                label = { Text("模拟路径") },
                trailingIcon = {
                    ExposedDropdownMenuDefaults.TrailingIcon(expanded = routeExpanded)
                },
                modifier = Modifier.fillMaxWidth()
                    .menuAnchor(),
            )
            ExposedDropdownMenu(
                expanded = routeExpanded,
                onDismissRequest = { routeExpanded = false },
            ) {
                DropdownMenuItem(
                    text = { Text("无路径") },
                    onClick = {
                        routeName = ""
                        profile.setRouteName(null)
                        routeExpanded = false
                    },
                )
                routeNames.forEach { name ->
                    DropdownMenuItem(
                        text = { Text(name) },
                        onClick = {
                            routeName = name
                            profile.setRouteName(name)
                            routeExpanded = false
                        },
                    )
                }
            }
        }

        LabeledCheckbox(
            checked = profile.autoStop(),
            text = "路径结束自动 STOP",
            onCheckedChange = profile::setAutoStop,
        )

        FlowRow(
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Button(
                onClick = { robot.opModeBackend.externalInit(selectedName) },
                enabled = snapshot.opMode.phase == OpModeLifecycleService.Phase.STOPPED,
            ) { Text("INIT") }
            Button(
                onClick = { robot.opModeBackend.externalStart() },
                enabled = snapshot.opMode.phase == OpModeLifecycleService.Phase.INIT,
            ) { Text("START") }
            Button(
                onClick = { robot.opModeBackend.externalStop() },
                enabled = snapshot.opMode.phase != OpModeLifecycleService.Phase.STOPPED,
            ) { Text("STOP") }
        }

        HorizontalDivider()
        ValueRow("Lifecycle", snapshot.opMode.phase.toString())
        ValueRow("Active", snapshot.opMode.activeName ?: "—")
        ValueRow(
            "Execution",
            snapshot.execution.state.toString()
                + (snapshot.execution.subject?.let { " · $it" } ?: ""),
        )
        ValueRow(
            "Route",
            snapshot.simulation.routeName?.let {
                String.format(
                    Locale.US,
                    "%s · %.1f / %.1f s",
                    it,
                    snapshot.simulation.routeSeconds,
                    snapshot.simulation.routeTotalSeconds,
                )
            } ?: "—",
        )

        if (routeName.isNotBlank()) {
            OutlinedButton(
                onClick = { robot.simulation.snapToRouteStart(routeName) },
                modifier = Modifier.fillMaxWidth(),
            ) {
                Text("机器人移到路径起点")
            }
        }

        PoseEditor(robot, snapshot)

        Text(
            "这里的生命周期按钮模拟 Driver Station / Dashboard 等其他控制端。"
                + "AzConductor 应通过真实 SSE 自动同步这些变化。",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun PoseEditor(robot: MockRobotRuntime, snapshot: UiSnapshot) {
    var x by remember { mutableStateOf(String.format(Locale.US, "%.1f", snapshot.simulation.x)) }
    var y by remember { mutableStateOf(String.format(Locale.US, "%.1f", snapshot.simulation.y)) }
    var heading by remember {
        mutableStateOf(String.format(Locale.US, "%.1f", snapshot.simulation.heading))
    }

    Text("手动 Pose", style = MaterialTheme.typography.titleSmall)
    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        CompactNumberField("X", x, { x = it }, Modifier.weight(1f))
        CompactNumberField("Y", y, { y = it }, Modifier.weight(1f))
        CompactNumberField("H", heading, { heading = it }, Modifier.weight(1f))
    }
    OutlinedButton(
        onClick = {
            robot.simulation.setPose(
                x.toDoubleOrNull() ?: snapshot.simulation.x,
                y.toDoubleOrNull() ?: snapshot.simulation.y,
                heading.toDoubleOrNull() ?: snapshot.simulation.heading,
            )
        },
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text("应用 Pose")
    }
}

@Composable
private fun SimulationControls(robot: MockRobotRuntime, snapshot: UiSnapshot) {
    var positionSigma by remember {
        mutableStateOf(robot.settings.positionNoiseSigma().toString())
    }
    var headingSigma by remember {
        mutableStateOf(robot.settings.headingNoiseSigma().toString())
    }
    var correction by remember {
        mutableStateOf(robot.settings.correctionSeconds().toString())
    }
    var speed by remember {
        mutableStateOf(robot.settings.speedScale().toString())
    }
    var poseHz by remember { mutableStateOf(robot.settings.poseHz().toString()) }
    var randomSeed by remember { mutableStateOf(robot.settings.randomSeed().toString()) }

    Column(verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Text("跟踪仿真", style = MaterialTheme.typography.titleMedium)
        SettingField("位置随机扰动 σ (in)", positionSigma) {
            positionSigma = it
            it.toDoubleOrNull()?.let(robot.settings::setPositionNoiseSigma)
        }
        SettingField("朝向随机扰动 σ (°)", headingSigma) {
            headingSigma = it
            it.toDoubleOrNull()?.let(robot.settings::setHeadingNoiseSigma)
        }
        SettingField("拖拽/扰动纠偏时间常数 (s)", correction) {
            correction = it
            it.toDoubleOrNull()?.let(robot.settings::setCorrectionSeconds)
        }
        SettingField("自动运行速度倍率", speed) {
            speed = it
            it.toDoubleOrNull()?.let(robot.settings::setSpeedScale)
        }
        SettingField("Pose 发布频率 (1–60Hz)", poseHz) {
            poseHz = it
            it.toIntOrNull()?.let(robot.settings::setPoseHz)
        }

        LabeledCheckbox(
            checked = robot.settings.noiseEnabled(),
            text = "启用连续随机跟踪扰动",
            onCheckedChange = robot.settings::setNoiseEnabled,
        )
        LabeledCheckbox(
            checked = robot.settings.loopRoute(),
            text = "循环运行路径",
            onCheckedChange = robot.settings::setLoopRoute,
        )

        FlowRow(
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Button(onClick = { robot.simulation.addRandomKick() }) {
                Text("注入大扰动")
            }
            OutlinedButton(onClick = { robot.simulation.resetTrackingError() }) {
                Text("清除跟踪误差")
            }
        }

        SettingField("随机种子", randomSeed) { randomSeed = it }
        OutlinedButton(
            onClick = {
                randomSeed.toLongOrNull()?.let {
                    robot.settings.setRandomSeed(it)
                    robot.simulation.reseedNoise()
                }
            },
            modifier = Modifier.fillMaxWidth(),
        ) {
            Text("应用随机种子")
        }

        HorizontalDivider()
        ValueRow(
            "当前误差",
            String.format(Locale.US, "%.3f in", snapshot.simulation.trackingError),
        )
        ValueRow(
            "自动运行",
            if (snapshot.simulation.routeRunning) "RUNNING" else "IDLE",
        )
    }
}

@Composable
private fun NetworkControls(robot: MockRobotRuntime, snapshot: UiSnapshot) {
    var latency by remember { mutableStateOf(robot.settings.httpLatencyMs().toString()) }

    Column(verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Text("Network V2", style = MaterialTheme.typography.titleMedium)
        ValueRow("监听", "0.0.0.0:${MockRobotRuntime.PORT}")
        ValueRow("Protocol", "2")
        ValueRow(
            "Session",
            if (snapshot.sessionOwned) snapshot.sessionOwner ?: "owned" else "—",
        )

        SettingField("HTTP 人工延迟 (ms)", latency) {
            latency = it
            it.toIntOrNull()?.let(robot.settings::setHttpLatencyMs)
        }

        OutlinedButton(
            onClick = robot::disconnectClient,
            enabled = snapshot.sessionOwned,
            modifier = Modifier.fillMaxWidth(),
        ) {
            Text("强制断开当前 AzConductor Session")
        }

        HorizontalDivider()
        Text("机器人 Routes", style = MaterialTheme.typography.titleSmall)
        snapshot.routes.forEach { route ->
            ValueRow(route.name, "rev ${route.revision}")
        }
        OutlinedButton(
            onClick = { robot.routes.replaceWithDemoRoutes() },
            modifier = Modifier.fillMaxWidth(),
        ) {
            Text("恢复内置 Demo Routes")
        }
        OutlinedButton(
            onClick = {
                val directory = NioPath.of(
                    System.getProperty("user.home"),
                    ".azconductor",
                    "mockrobot",
                )
                runCatching {
                    Files.createDirectories(directory)
                    if (Desktop.isDesktopSupported()) {
                        Desktop.getDesktop().open(directory.toFile())
                    }
                }.onFailure {
                    MockLog.error("UI", "打开数据目录失败", it)
                }
            },
            modifier = Modifier.fillMaxWidth(),
        ) {
            Text("打开 MockRobot 数据目录")
        }

        Text(
            "HTTP Router、Session、SSE、revision/ETag/If-Match 与机器人端复用同一套源码。"
                + "MockRobot 不提供客户端特判。",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun LogPanel(logs: List<String>, modifier: Modifier = Modifier) {
    Card(modifier = modifier, shape = RoundedCornerShape(12.dp)) {
        Column(Modifier.fillMaxSize().padding(8.dp)) {
            Text("实时日志", fontWeight = FontWeight.SemiBold)
            HorizontalDivider(Modifier.padding(vertical = 5.dp))
            val scroll = rememberScrollState()
            LaunchedEffect(logs.size) {
                scroll.scrollTo(scroll.maxValue)
            }
            Column(modifier = Modifier.fillMaxSize().verticalScroll(scroll)) {
                logs.takeLast(300).forEach { line ->
                    Text(
                        line,
                        fontFamily = FontFamily.Monospace,
                        fontSize = 10.sp,
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis,
                    )
                }
            }
        }
    }
}

@Composable
private fun SettingField(label: String, value: String, onValueChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text(label) },
        singleLine = true,
        modifier = Modifier.fillMaxWidth(),
    )
}

@Composable
private fun CompactNumberField(
    label: String,
    value: String,
    onValueChange: (String) -> Unit,
    modifier: Modifier = Modifier,
) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text(label) },
        singleLine = true,
        modifier = modifier,
    )
}

@Composable
private fun LabeledCheckbox(
    checked: Boolean,
    text: String,
    onCheckedChange: (Boolean) -> Unit,
) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Checkbox(checked = checked, onCheckedChange = onCheckedChange)
        Text(text)
    }
}

@Composable
private fun ValueRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Spacer(Modifier.width(12.dp))
        Text(value, fontWeight = FontWeight.Medium)
    }
}

private fun selectedPreviewPlan(
    robot: MockRobotRuntime,
    snapshot: UiSnapshot,
): MockRoutePlan? {
    val activeName = snapshot.opMode.activeName ?: return null
    val profile = robot.opModeBackend.profile(activeName) ?: return null
    val routeName = profile.routeName() ?: return null
    val entry = robot.routes.get(routeName) ?: return null
    return runCatching { MockRoutePlan.parse(entry.json) }.getOrNull()
}
