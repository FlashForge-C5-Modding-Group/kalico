import types
from unittest import mock

from klippy.extras import usb_gcode as ug


def make_sys_block(tmp_path, disks):
    root = tmp_path / "block"
    root.mkdir()
    for disk, parts in disks.items():
        d = root / disk
        (d / "device").mkdir(parents=True)
        (d / "device" / "vendor").write_text("Generic \n")
        (d / "device" / "model").write_text("Flash Disk\n")
        (d / "size").write_text("7813120\n")
        for p in parts:
            (d / p).mkdir()
    return str(root)


def test_finds_first_partition_of_first_usb_disk(tmp_path):
    root = make_sys_block(tmp_path, {"sdb": ["sdb1"], "sda": ["sda2", "sda1"],
                                     "mmcblk0": ["mmcblk0p1"]})
    assert ug.find_usb_device(root) == ("/dev/sda1", "sda")


def test_unpartitioned_disk_is_used_whole(tmp_path):
    root = make_sys_block(tmp_path, {"sda": []})
    assert ug.find_usb_device(root) == ("/dev/sda", "sda")


def test_no_usb_disk(tmp_path):
    root = make_sys_block(tmp_path, {"mmcblk0": ["mmcblk0p1"]})
    assert ug.find_usb_device(root) == (None, None)
    assert ug.find_usb_device(str(tmp_path / "missing")) == (None, None)


def test_describe_disk(tmp_path):
    root = make_sys_block(tmp_path, {"sda": ["sda1"]})
    assert ug.describe_disk("sda", root) == "Generic Flash Disk (4.0 GB)"


def test_read_mounts(tmp_path):
    p = tmp_path / "mounts"
    p.write_text("/dev/root / squashfs ro 0 0\n"
                 "/dev/sda1 /mnt vfat rw 0 0\n"
                 "/dev/sda1 /usr/data/gcodes vfat rw 0 0\n")
    mounts = ug.read_mounts(str(p))
    assert mounts["/mnt"] == "/dev/sda1"
    assert mounts["/usr/data/gcodes"] == "/dev/sda1"


class Config:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def get(self, name, default=None):
        return default

    def getfloat(self, name, default=None, **kw):
        return default


def make_unit(state="standby"):
    gcode = mock.Mock()
    reactor = mock.Mock(NOW=0.0, monotonic=mock.Mock(return_value=10.0))
    stats = mock.Mock(get_status=mock.Mock(return_value={"state": state}))
    vsd = types.SimpleNamespace(sdcard_dirname="/usr/data/gcodes")
    objects = {"gcode": gcode, "virtual_sdcard": vsd, "print_stats": stats}
    printer = mock.Mock(
        get_reactor=mock.Mock(return_value=reactor),
        lookup_object=mock.Mock(side_effect=lambda n, d=None: objects.get(n, d)))
    unit = ug.UsbGcode(Config(printer))
    with mock.patch.object(ug, "read_mounts", return_value={}):
        unit._handle_ready()
    return unit, gcode, stats


def raw_lines(gcode):
    return [c.args[0] for c in gcode.respond_raw.call_args_list]


def test_insert_mounts_then_asks(monkeypatch):
    unit, gcode, _ = make_unit()
    monkeypatch.setattr(ug, "find_usb_device", lambda: ("/dev/sda1", "sda"))
    monkeypatch.setattr(ug, "describe_disk", lambda d: "Stick (4.0 GB)")
    proc = mock.Mock(poll=mock.Mock(return_value=0), returncode=0)
    popen = mock.Mock(return_value=proc)
    monkeypatch.setattr(ug.subprocess, "Popen", popen)

    unit._poll(0.0)
    assert popen.call_args.args[0] == [
        "mount", "-t", "vfat", "-o", "iocharset=utf8,noatime",
        "/dev/sda1", "/mnt"]
    unit._poll(0.2)
    assert unit.mounted
    assert "// action:prompt_show" in raw_lines(gcode)
    gcode.respond_raw.reset_mock()
    unit._poll(2.2)
    assert raw_lines(gcode) == []  # asked once per insert


def test_waits_for_the_print_to_end(monkeypatch):
    unit, gcode, stats = make_unit(state="printing")
    monkeypatch.setattr(ug, "find_usb_device", lambda: ("/dev/sda1", "sda"))
    monkeypatch.setattr(ug, "describe_disk", lambda d: "Stick")
    popen = mock.Mock()
    monkeypatch.setattr(ug.subprocess, "Popen", popen)
    unit._poll(0.0)
    popen.assert_not_called()
    stats.get_status.return_value = {"state": "complete"}
    unit._poll(2.0)
    popen.assert_called_once()


def test_use_bind_mounts_over_the_gcode_folder(monkeypatch):
    unit, gcode, _ = make_unit()
    unit.mounted, unit.label = True, "Stick"
    check = mock.Mock(return_value=b"")
    monkeypatch.setattr(ug.subprocess, "check_output", check)
    gcmd = mock.Mock(error=RuntimeError)
    unit.cmd_USB_GCODE_USE(gcmd)
    assert check.call_args.args[0] == [
        "mount", "--bind", "/mnt", "/usr/data/gcodes"]
    assert unit.active


def test_use_refused_while_printing(monkeypatch):
    unit, gcode, stats = make_unit(state="printing")
    unit.mounted = True
    check = mock.Mock()
    monkeypatch.setattr(ug.subprocess, "check_output", check)
    gcmd = mock.Mock(error=RuntimeError)
    try:
        unit.cmd_USB_GCODE_USE(gcmd)
    except RuntimeError:
        pass
    else:
        raise AssertionError("expected an error")
    check.assert_not_called()


def test_removal_unmounts_and_returns_to_internal(monkeypatch):
    unit, gcode, _ = make_unit()
    unit.device, unit.disk, unit.label = "/dev/sda1", "sda", "Stick"
    unit.mounted = unit.active = True
    monkeypatch.setattr(ug, "find_usb_device", lambda: (None, None))
    monkeypatch.setattr(ug, "read_mounts", lambda: {
        "/mnt": "/dev/sda1", "/usr/data/gcodes": "/dev/sda1"})
    call = mock.Mock(return_value=0)
    monkeypatch.setattr(ug.subprocess, "call", call)
    unit._poll(0.0)
    assert [c.args[0] for c in call.call_args_list] == [
        ["umount", "-l", "/usr/data/gcodes"], ["umount", "-l", "/mnt"]]
    assert not unit.active and unit.device is None
    assert "internal storage again" in gcode.respond_info.call_args.args[0]


def test_restart_picks_up_an_active_switch(monkeypatch):
    unit, gcode, _ = make_unit()
    monkeypatch.setattr(ug, "describe_disk", lambda d: "Stick")
    with mock.patch.object(ug, "read_mounts", return_value={
            "/mnt": "/dev/sda1", "/usr/data/gcodes": "/dev/sda1"}):
        unit._handle_ready()
    assert unit.mounted and unit.active and unit.disk == "sda"
