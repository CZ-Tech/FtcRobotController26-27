## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. It contains this team's robot subsystems and OpModes;
the SDK sample-copying instructions below remain useful when adding new OpModes.

## Subsystems, aggregate commands, and task threads

- Subsystems are ordinary Java objects. A direct method call runs synchronously on its
  caller, including direct hardware/status access. `SubsystemLifecycle.periodic()` and
  `onShutdown()` participate in the OpMode lifecycle; they do not schedule commands.
- Only methods explicitly marked `@RemoteCommand` are published. Keep `@Param` on exposed
  parameters. The annotation controls exposure, not threading: every external HTTP/route
  invocation starts a managed task by default. No async annotation, generated wrapper,
  scheduler call, or query registry is needed.
- An aggregate is an ordinary published method in `AutoCommands` (or another public
  subsystem field). Write direct subsystem calls, `Thread.sleep(...)`, conditions and
  interruptible polling loops. Calls to other aggregate methods stay on the same thread.
  For example, `auto.shoot(rpm, spinupMillis, feedMillis)` directly spins up, sleeps,
  opens the valve, sleeps, then closes/stops. `intaker.setIntakerPower(power, time)` now
  waits synchronously and stops before returning; direct OpMode callers must not use a
  timed/blocking method on the path loop if they want movement to continue.
- One waypoint's `commands` list runs in order on one task thread. Different waypoints
  and separate HTTP invocations may overlap, without FIFO completion order, ownership,
  priority, or conflict arbitration. An accepted HTTP command is not necessarily completed.
- STOP closes task ingress, interrupts workers, and waits up to one second total before
  hardware cleanup. Commands must propagate `InterruptedException`, or restore interruption
  and return. Polling loops need an interrupt check and a timeout. Never swallow cancellation
  and keep working. Built-in subsystem entry points reject cancelled-task access; direct
  raw SDK references cannot be revoked. A non-cooperating worker is reported, never forcibly
  killed with `Thread.stop()`. Shared application state needs `volatile`, atomics or locks.
- Worker failures abort the OpMode via its update loop and cancel sibling tasks; no implicit
  retry. Normal route completion stops movement, seals ingress, and keeps updating until
  started commands finish. STOP/failure cancels rather than waiting for normal completion.
- `auto.stopAll()` cancels sibling tasks and legacy flows, then stops intake/shooter; it
  does not cancel its own caller or stop the route/drivetrain. Later commands in that same
  caller can still execute. It is not an emergency-stop API.
- `ActionSequence` remains an optional legacy control-loop facility. Its callbacks must
  not block. `clearFlows()` clears those flows only; it does not cancel command threads or
  automatically stop hardware. The old generated Scheduler model remains removed.

### Waypoint command lists

New route JSON uses a top-level format version and frame array. Bare arrays remain
readable as legacy v1; an object must have numeric `version: 2` and an array `frames`.
Unknown versions and malformed envelopes are rejected on upload and before movement.
The format version is independent of the HTTP route revision (`If-Match`).
`commands` is optional; commands fire when the point completes without pausing movement.
The first geometric point sets the initial pose.

```json
{
  "version": 2,
  "frames": [
    {
      "x": 0, "y": 0, "dx": 24, "dy": 0, "heading": 0,
      "commands": [
        {"name": "intaker.intakerIn", "params": ["1"]},
        {"name": "auto.waitMillis", "params": ["1000"]},
        {"name": "intaker.intakerArmStop", "params": []}
      ]
    },
    {
      "x": 36, "y": 0, "dx": 24, "dy": 0, "heading": 90,
      "commands": [
        {"name": "auto.shoot", "params": ["6000", "1000", "500"]}
      ]
    }
  ]
}
```

`auto.waitMillis` is just a normal Java command, not an interpreter instruction.
Arguments remain ordered strings converted using the existing command catalog. Each list
is fully resolved before a single worker is submitted; an invalid later entry cannot start
an earlier one. Legacy `command`/`commandParams` stays supported. Do not combine a non-empty
legacy command with a non-empty `commands` list. Movement `{"wait": seconds}` frames remain
supported and are independent of command-thread waits.

### Command signatures and optional parameters

`GET /api/v2/commands` retains the legacy `paramNames`, `paramTypes`, and `paramDefaults`
for the longest accepted signature. It also publishes:

- `signatures`: every accepted invocation, sorted by argument count, each with its own
  `paramNames`, `paramTypes`, and `paramDefaults`. This includes shorter invocations
  allowed by overloads or omitted trailing `@Param(defaultValue = "...")` arguments.
- `acceptedArgCounts`: the exact accepted argument counts, in ascending order.
- `hasUnifiedSignature`: true only when counts are continuous and all shorter
  signatures have the same parameter-name and type prefixes as the longest signature.
- `requiredParamCount` and `paramRequired`: the minimum argument count and per-parameter
  required flags for a unified signature. Both are JSON `null` when signatures cannot
  be merged; clients should then offer the individual signatures instead.

For example, `intakerIn()` plus `intakerIn(@Param("power") double power)` publishes
`acceptedArgCounts: [0, 1]`, `requiredParamCount: 0`, and `paramRequired: [false]`.
`paramDefaults: [null]` means the default value is unknown, **not** that power is required.
Only explicit annotation defaults are published; never infer defaults from method bodies
or replace omitted arguments with client-side values. Send only the supplied argument
prefix so the binding selects the real overload (or supplies explicit annotation defaults).
Signatures accepting only 1 and 3 arguments are not merged, and same-arity overload
conflicts remain unpublished.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""