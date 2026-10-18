# Debugging

This document describes some of the Kalico debugging tools.

## Running the regression tests

The main Kalico GitHub repository uses "github actions" to run a
series of regression tests. It can be useful to run some of these
tests locally.

The source code "whitespace check" can be run with:
```
./scripts/check_whitespace.sh
```

The Klippy regression test suite requires "data dictionaries" from
many platforms. The easiest way to obtain them is to
[download them from github](https://github.com/Klipper3d/klipper/issues/1438).
Once the data dictionaries are downloaded, use the following to run
the regression suite:
```
tar xfz klipper-dict-20??????.tar.gz
~/klippy-env/bin/python ~/klipper/scripts/test_klippy.py -d dict/ ~/klipper/test/klippy/*.test
```

## Manually sending commands to the micro-controller

Normally, the host klippy.py process would be used to translate gcode
commands to Kalico micro-controller commands. However, it's also
possible to manually send these MCU commands (functions marked with
the DECL_COMMAND() macro in the Kalico source code). To do so, run:

```
~/klippy-env/bin/python ./klippy/console.py /tmp/pseudoserial
```

See the "HELP" command within the tool for more information on its
functionality.

Some command-line options are available. For more information run:
`~/klippy-env/bin/python ./klippy/console.py --help`

## Translating gcode files to micro-controller commands

The Klippy host code can run in a batch mode to produce the low-level
micro-controller commands associated with a gcode file. Inspecting
these low-level commands is useful when trying to understand the
actions of the low-level hardware. It can also be useful to compare
the difference in micro-controller commands after a code change.

To run Klippy in this batch mode, there is a one time step necessary
to generate the micro-controller "data dictionary". This is done by
compiling the micro-controller code to obtain the **out/klipper.dict**
file:

```
make menuconfig
make
```

Once the above is done it is possible to run Kalico in batch mode
(see [installation](Installation.md) for the steps necessary to build
the python virtual environment and a printer.cfg file):

```
~/klippy-env/bin/python ./klippy/klippy.py ~/printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict
```

The above will produce a file **test.serial** with the binary serial
output. This output can be translated to readable text with:

```
~/klippy-env/bin/python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt
```

The resulting file **test.txt** contains a human readable list of
micro-controller commands.

The batch mode disables certain response / request commands in order
to function. As a result, there will be some differences between
actual commands and the above output. The generated data is useful for
testing and inspection; it is not useful for sending to a real
micro-controller.

## Motion timing diagnostics

The opt-in host capture helps distinguish late motion generation from
late delivery to an MCU. It does not change scheduling constants, queue
eligibility, clocks, or safety thresholds. `Timer too close` remains a
nonfatal warning; independent shutdown checks such as `Stepper too far
in past` remain in effect. Warning reports and this instrumentation are
not a fix for the original late-step failure. A causal patch requires
evidence of the bottleneck and a supervised verification afterward.

### Deploying the matching host code

Deploy the reviewed host revision through the installation's existing
source-update mechanism. Include the Python sources and matching native
C helper sources, headers, and CFFI declarations together; do not deploy
only the Python files or reuse an incompatible helper binary. Record
the actual deployed source revision and helper build identity with the
capture. The helper uses the existing host-side rebuild path when its
sources change; the host must have the normal compiler and Python/CFFI
build prerequisites. Confirm the matching helper rebuild/load succeeds
before attempting the capture. A helper build or compatibility error is
a deployment error, not a reason to silently omit native diagnostics.

Restart the Klippy host service after updating, using the installation's
service name. For the standard service convention used in the
[migration instructions](Migrating_from_Klipper.md), this is:

```
sudo systemctl restart klipper
```

This is a host-only change: no MCU firmware update or flashing is
required, and no printer configuration section or configuration
overwrite is needed. Do not run `SAVE_CONFIG` for this feature. Use the
known printer host and existing installation paths; the capture
procedure does not prescribe a remote address or a deployment script.

### Supervised reproduction

Capture is disabled by default. While idle, enable it with
`SET_MOTION_TIMING_DIAGNOSTICS ENABLE=1`. Enabling clears previous
records and enables all currently connected MCU serial queues.
`ENABLE=0` stops recording without deleting history; re-enabling starts
fresh. Changing the setting during a print, including while paused, is
rejected with `Cannot change motion timing diagnostics during a print`.
No configuration section is added.

The operator must confirm bed/nozzle clearance and supervise the
existing print. The print's normal startup may heat, home, and move the
printer. For the purge-line failure reproduction, the operator enters:

```gcode
SET_MOTION_TIMING_DIAGNOSTICS ENABLE=1
SDCARD_PRINT_FILE FILENAME="Cartographer V4 - Standard.step_PLA_18m54s.gcode"
```

After completion or failure, **before restarting the host**, enter:

```gcode
GET_TIMER_TOO_CLOSE_STATS
DUMP_MOTION_TIMING_DIAGNOSTICS
```

Both query commands remain available after shutdown. Save the complete
resulting printer log and console report, including the bounded capture
and records preceding the warning/failure, not only the final error
line. Reports and retained snapshots are in memory and are lost on host
restart. Do not re-enable capture before saving it, because doing so
clears the previous records. If the failure does not reproduce, retain
the successful capture; an uneventful run does not establish that the
original fault is fixed.

### Capture contents and interpretation

The Python ring retains the newest 128 flush/generation records. The
native serial queue retains the newest 128 enqueue/message-build
records per MCU, under its existing queue lock. Collection does not
allocate a new object per step. Python flush records distinguish an
expected invocation deadline from actual entry; externally kicked or
disabled timer invocations have no expected deadline. Generation
records separate time spent in Python flush callbacks from synchronous
C step generation/emission and include the requested/effective flush
and generation times.

Each warning can snapshot the numeric rings and non-destructive raw
serial history (up to 100 sent and 100 received messages per MCU).
The first warning snapshot is preserved; the latest snapshot refreshes
at most once per monotonic second. This snapshot throttling never
throttles the warning statistics. Raw message bytes and original send/
receive timestamps are retained in memory, so subsequent traffic
overwriting live history cannot erase a saved warning snapshot.
Messages are not decoded and bulk traces are not formatted/logged in
the warning callback while motion continues.

Print completion, cancellation, error, shutdown, and explicit dumps
capture current rings/history and emit a log line beginning
`MOTION_TIMING_DIAGNOSTICS ` followed by one JSON object. It contains
the trigger, host timestamp, live Python ring, MCU-name-to-serial-record
map, and retained first/latest warning snapshots (null when absent).
At output time, raw bytes are represented as hexadecimal and decoded
with the MCU's existing message parser for readable commands such as
`queue_step`. An undecodable record retains its hex bytes and timestamps
and is marked undecodable rather than discarding the rest of the
capture. Disconnected MCUs report unavailable live serial history
without preventing other MCU records or retained snapshots from being
dumped. Each dump also logs each MCU's live configuration, clock-sync,
and available serial debug information.

Native `kind=1` records mark enqueue; `kind=2` records mark outgoing
message-build. Their `req_clock` is the queue's priority/need time,
**not universally a physical first-step deadline**. Zero request/minimum
clocks denote no finite positive scheduled request and must not be
interpreted as a deadline miss. Correlate finite request clocks with
the retained decoded `queue_step` commands and clock conversion fields,
including 32-bit wrap, before reconstructing a missed step deadline.
In the live serial send dump, the **first displayed time is estimated
wire completion, not measured arrival at the MCU**; the other time is
the original message-build/send timestamp. Do not reverse those times
or treat their difference as a measured MCU latency.

Compare flush invocation lateness, Python callback duration,
C generation/emission duration, and enqueue-versus-build timing to
identify the demonstrated bottleneck. High CPU load alone does not
identify the cause. Preserve the warning's original host receipt time:
[`print_stats.timer_too_close`](Status_Reference.md#print_stats)
uses scheduled-layer/receive-time attribution, not physical layer
position. Known startup is layer 0; absent slicer metadata is unknown.
Elapsed warning times include pauses, and the completed report remains
queryable until the next print starts or the host restarts.

## Motion analysis and data logging

Kalico supports logging its internal motion history, which can be
later analyzed. To use this feature, Kalico must be started with the
[API Server](API_Server.md) enabled.

Data logging is enabled with the `data_logger.py` tool. For example:
```
~/klipper/scripts/motan/data_logger.py /tmp/klippy_uds mylog
```

This command will connect to the Kalico API Server, subscribe to
status and motion information, and log the results. Two files are
generated - a compressed data file and an index file (eg,
`mylog.json.gz` and `mylog.index.gz`). After starting the logging, it
is possible to complete prints and other actions - the logging will
continue in the background. When done logging, hit `ctrl-c` to exit
from the `data_logger.py` tool.

The resulting files can be read and graphed using the `motan_graph.py`
tool. To generate graphs on a Raspberry Pi, a one time step is
necessary to install the "matplotlib" package:
```
sudo apt-get update
sudo apt-get install python-matplotlib
```
However, it may be more convenient to copy the data files to a desktop
class machine along with the Python code in the `scripts/motan/`
directory. The motion analysis scripts should run on any machine with
a recent version of [Python](https://python.org) and
[Matplotlib](https://matplotlib.org/) installed.

Graphs can be generated with a command like the following:
```
~/klipper/scripts/motan/motan_graph.py mylog -o mygraph.png
```

One can use the `-g` option to specify the datasets to graph (it takes
a Python literal containing a list of lists). For example:
```
~/klipper/scripts/motan/motan_graph.py mylog -g '[["trapq(toolhead,velocity)"], ["trapq(toolhead,accel)"]]'
```

The list of available datasets can be found using the `-l` option -
for example:
```
~/klipper/scripts/motan/motan_graph.py -l
```

It is also possible to specify matplotlib plot options for each
dataset:
```
~/klipper/scripts/motan/motan_graph.py mylog -g '[["trapq(toolhead,velocity)?color=red&alpha=0.4"]]'
```
Many matplotlib options are available; some examples are "color",
"label", "alpha", and "linestyle".

The `motan_graph.py` tool supports several other command-line
options - use the `--help` option to see a list. It may also be
convenient to view/modify the
[motan_graph.py](../scripts/motan/motan_graph.py) script itself.

The raw data logs produced by the `data_logger.py` tool follow the
format described in the [API Server](API_Server.md). It may be useful
to inspect the data with a Unix command like the following:
`gunzip < mylog.json.gz | tr '\03' '\n' | less`

## Generating load graphs

The Klippy log file (/tmp/klippy.log) stores statistics on bandwidth,
micro-controller load, and host buffer load. It can be useful to graph
these statistics after a print.

To generate a graph, a one time step is necessary to install the
"matplotlib" package:

```
sudo apt-get update
sudo apt-get install python-matplotlib
```

Then graphs can be produced with:

```
~/klipper/scripts/graphstats.py /tmp/klippy.log -o loadgraph.png
```

One can then view the resulting **loadgraph.png** file.

Different graphs can be produced. For more information run:
`~/klipper/scripts/graphstats.py --help`

## Extracting information from the klippy.log file

The Klippy log file (/tmp/klippy.log) also contains debugging
information. There is a logextract.py script that may be useful when
analyzing a micro-controller shutdown or similar problem. It is
typically run with something like:

```
mkdir work_directory
cd work_directory
cp /tmp/klippy.log .
~/klipper/scripts/logextract.py ./klippy.log
```

The script will extract the printer config file and will extract MCU
shutdown information. The information dumps from an MCU shutdown (if
present) will be reordered by timestamp to assist in diagnosing cause
and effect scenarios.

## Testing with simulavr

The [simulavr](http://www.nongnu.org/simulavr/) tool enables one to
simulate an Atmel ATmega micro-controller. This section describes how
one can run test gcode files through simulavr. It is recommended to
run this on a desktop class machine (not a Raspberry Pi) as it does
require significant cpu to run efficiently.

To use simulavr, download the simulavr package and compile with python
support. Note that the build system may need to have some packages (such as
swig) installed in order to build the python module.

```
git clone git://git.savannah.nongnu.org/simulavr.git
cd simulavr
make python
make build
```
Make sure a file like **./build/pysimulavr/_pysimulavr.*.so** is present
after the above compilation:
```
ls ./build/pysimulavr/_pysimulavr.*.so
```
This command should report a specific file (e.g.
**./build/pysimulavr/_pysimulavr.cpython-39-x86_64-linux-gnu.so**) and
not an error.

If you are on a Debian-based system (Debian, Ubuntu, etc.) you can
install the following packages and generate *.deb files for system-wide
installation of simulavr:
```
sudo apt update
sudo apt install g++ make cmake swig rst2pdf help2man texinfo
make cfgclean python debian
sudo dpkg -i build/debian/python3-simulavr*.deb
```

To compile Kalico for use in simulavr, run:

```
cd /path/to/kalico
make menuconfig
```

and compile the micro-controller software for an AVR atmega644p and
select SIMULAVR software emulation support. Then one can compile
Kalico (run `make`) and then start the simulation with:

```
PYTHONPATH=/path/to/simulavr/build/pysimulavr/ ./scripts/avrsim.py out/klipper.elf
```
Note that if you have installed python3-simulavr system-wide, you do
not need to set `PYTHONPATH`, and can simply run the simulator as
```
./scripts/avrsim.py out/klipper.elf
```

Then, with simulavr running in another window, one can run the
following to read gcode from a file (eg, "test.gcode"), process it
with Klippy, and send it to Kalico running in simulavr (see
[installation](Installation.md) for the steps necessary to build the
python virtual environment):

```
~/klippy-env/bin/python ./klippy/klippy.py config/generic-simulavr.cfg -i test.gcode -v
```

### Using simulavr with gtkwave

One useful feature of simulavr is its ability to create signal wave
generation files with the exact timing of events. To do this, follow
the directions above, but run avrsim.py with a command-line like the
following:

```
PYTHONPATH=/path/to/simulavr/src/python/ ./scripts/avrsim.py out/klipper.elf -t PORTA.PORT,PORTC.PORT
```

The above would create a file **avrsim.vcd** with information on each
change to the GPIOs on PORTA and PORTB. This could then be viewed
using gtkwave with:

```
gtkwave avrsim.vcd
```
