# Offer a USB drive as the G-code folder when one is plugged in
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import subprocess

SYS_BLOCK = "/sys/block"
PROC_MOUNTS = "/proc/mounts"


def find_usb_device(sys_block=SYS_BLOCK):
    """First USB disk's first partition (or the disk itself): (dev, disk)."""
    try:
        disks = sorted(d for d in os.listdir(sys_block) if d.startswith("sd"))
    except OSError:
        return None, None
    for disk in disks:
        try:
            parts = sorted(
                p
                for p in os.listdir(os.path.join(sys_block, disk))
                if p.startswith(disk)
            )
        except OSError:
            continue
        return "/dev/" + (parts[0] if parts else disk), disk
    return None, None


def read_mounts(proc_mounts=PROC_MOUNTS):
    """Mount point -> source device."""
    mounts = {}
    try:
        with open(proc_mounts) as f:
            for line in f:
                fields = line.split()
                if len(fields) >= 2:
                    mounts[fields[1].replace("\\040", " ")] = fields[0]
    except OSError:
        pass
    return mounts


def _read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def describe_disk(disk, sys_block=SYS_BLOCK):
    base = os.path.join(sys_block, disk)
    name = " ".join(
        s
        for s in (_read(base + "/device/vendor"), _read(base + "/device/model"))
        if s
    )
    try:
        size_gb = int(_read(base + "/size", "0")) * 512 / 1e9
    except ValueError:
        size_gb = 0.0
    label = name or "USB drive"
    if size_gb:
        label += " (%.1f GB)" % (size_gb,)
    return label


class UsbGcode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.mount_point = config.get("mount_point", "/mnt")
        self.poll_interval = config.getfloat(
            "poll_interval", 2.0, minval=0.5
        )
        self.mount_options = config.get(
            "mount_options", "iocharset=utf8,noatime,flush"
        )
        self.gcode_path = None
        self.print_stats = None
        self.device = self.disk = None
        self.label = ""
        self.mounted = self.active = self.asked = False
        self.mount_proc = None
        self.mount_attempt = 0
        self.timer = None
        for name in ("USE", "INTERNAL", "KEEP"):
            cmd = "USB_GCODE_" + name
            self.gcode.register_command(
                cmd,
                getattr(self, "cmd_" + cmd),
                desc=getattr(self, "cmd_%s_help" % cmd),
            )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        vsd = self.printer.lookup_object("virtual_sdcard", None)
        if vsd is None:
            logging.info("usb_gcode: no [virtual_sdcard], disabled")
            return
        self.gcode_path = vsd.sdcard_dirname
        self.print_stats = self.printer.lookup_object("print_stats", None)
        # Mounts outlive a Klipper restart
        mounts = read_mounts()
        source = mounts.get(self.mount_point, "")
        if source.startswith("/dev/sd"):
            self.device = source
            self.disk = os.path.basename(source).rstrip("0123456789")
            self.label = describe_disk(self.disk)
            self.mounted = True
            self.active = mounts.get(self.gcode_path, "").startswith("/dev/sd")
            self.asked = self.active
        self.timer = self.reactor.register_timer(self._poll, self.reactor.NOW)

    def _busy(self, eventtime):
        if self.print_stats is None:
            return False
        state = self.print_stats.get_status(eventtime)["state"]
        return state in ("printing", "paused")

    def _poll(self, eventtime):
        next_time = eventtime + self.poll_interval
        if self.mount_proc is not None:
            if self.mount_proc.poll() is None:
                return eventtime + 0.2
            self._mount_done(self.mount_proc.returncode)
            self.mount_proc = None
        dev, disk = find_usb_device()
        if self.device is not None and dev != self.device:
            self._removed()
        if dev is None:
            return next_time
        if self.device is None:
            self.device, self.disk = dev, disk
            self.label = describe_disk(disk)
            self.mount_attempt = 0
            self.asked = False
        # Nothing that can block the host while printing
        if self._busy(eventtime):
            return next_time
        if not self.mounted and self.mount_attempt < 2:
            self._start_mount()
            return eventtime + 0.2
        if self.mounted and not self.active and not self.asked:
            self.asked = True
            self._prompt()
        return next_time

    def _start_mount(self):
        # vfat first, as the stock boot script does; then any type
        if self.mount_attempt == 0:
            cmd = ["mount", "-t", "vfat", "-o", self.mount_options]
        else:
            cmd = ["mount", "-o", "noatime,sync"]
        cmd += [self.device, self.mount_point]
        self.mount_attempt += 1
        try:
            self.mount_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
            )
        except OSError as e:
            logging.warning("usb_gcode: %s failed: %s", " ".join(cmd), e)
            self.mount_proc = None

    def _mount_done(self, returncode):
        if returncode == 0:
            self.mounted = True
            logging.info(
                "usb_gcode: %s mounted at %s", self.device, self.mount_point
            )
            return
        err = self.mount_proc.stderr.read().decode(errors="replace").strip()
        logging.info("usb_gcode: mount of %s failed: %s", self.device, err)
        if self.mount_attempt >= 2:
            self.gcode.respond_info(
                "%s could not be mounted: %s" % (self.label, err or "unknown")
            )

    def _removed(self):
        was_active = self.active
        for path in (self.gcode_path, self.mount_point):
            if path and read_mounts().get(path, "").startswith("/dev/sd"):
                subprocess.call(
                    ["umount", "-l", path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
        self._close_prompt()
        if was_active:
            self.gcode.respond_info(
                "%s removed: the G-code folder is internal storage again"
                % (self.label,)
            )
        self.device = self.disk = None
        self.label = ""
        self.mounted = self.active = self.asked = False

    def _prompt(self):
        respond = self.gcode.respond_raw
        respond("// action:prompt_begin USB drive")
        respond(
            "// action:prompt_text %s is plugged in. Use it as the G-code "
            "folder?" % (self.label,)
        )
        respond(
            "// action:prompt_text Your internal files come back when you "
            "switch back or remove the drive."
        )
        respond("// action:prompt_footer_button Keep internal|USB_GCODE_KEEP")
        respond("// action:prompt_footer_button Use USB|USB_GCODE_USE|primary")
        respond("// action:prompt_show")

    def _close_prompt(self):
        self.gcode.respond_raw("// action:prompt_end")

    def _check_idle(self, gcmd):
        if self._busy(self.reactor.monotonic()):
            raise gcmd.error("Not while a print is running or paused")

    cmd_USB_GCODE_USE_help = "Use the plugged-in USB drive as the G-code folder"

    def cmd_USB_GCODE_USE(self, gcmd):
        self._close_prompt()
        if not self.mounted:
            raise gcmd.error("No USB drive is mounted")
        if self.active:
            gcmd.respond_info("%s is already the G-code folder" % (self.label,))
            return
        self._check_idle(gcmd)
        try:
            subprocess.check_output(
                ["mount", "--bind", self.mount_point, self.gcode_path],
                stderr=subprocess.STDOUT,
                timeout=10,
            )
        except (OSError, subprocess.SubprocessError) as e:
            out = getattr(e, "output", b"") or b""
            raise gcmd.error(
                "Could not switch to the USB drive: %s"
                % (out.decode(errors="replace").strip() or e,)
            )
        self.active = True
        gcmd.respond_info(
            "%s is now the G-code folder. Refresh the file list to see its "
            "files." % (self.label,)
        )

    cmd_USB_GCODE_INTERNAL_help = "Switch the G-code folder back to internal storage"

    def cmd_USB_GCODE_INTERNAL(self, gcmd):
        self._close_prompt()
        if not self.active:
            gcmd.respond_info("The G-code folder is internal storage already")
            return
        self._check_idle(gcmd)
        try:
            subprocess.check_output(
                ["umount", self.gcode_path],
                stderr=subprocess.STDOUT,
                timeout=10,
            )
        except (OSError, subprocess.SubprocessError) as e:
            out = getattr(e, "output", b"") or b""
            raise gcmd.error(
                "Could not switch back: %s"
                % (out.decode(errors="replace").strip() or e,)
            )
        self.active = False
        gcmd.respond_info("The G-code folder is internal storage again")

    cmd_USB_GCODE_KEEP_help = "Keep internal storage as the G-code folder"

    def cmd_USB_GCODE_KEEP(self, gcmd):
        self._close_prompt()
        if self.mounted and not self.active:
            gcmd.respond_info(
                "Keeping internal storage. USB_GCODE_USE switches to %s."
                % (self.label,)
            )

    def get_status(self, eventtime):
        return {
            "inserted": self.device is not None,
            "mounted": self.mounted,
            "active": self.active,
            "device": self.device or "",
            "label": self.label,
            "mount_point": self.mount_point,
        }


def load_config(config):
    return UsbGcode(config)
