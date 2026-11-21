# Running without the ACE

When an `[AFC_ACE]` / `[AFC_ACE2]` unit is configured but no ACE is plugged in, Klipper starts
as if the ACE weren't configured: each head keeps its standalone lane (`extruder0`-`extruder3`
from `printer.afc-standalone.cfg`) and its `T0`-`T3`, so the printer works like a stock C5.

## How it decides

At startup (klippy:connect) the unit looks for its ACE on USB without opening it:

- `serial_port: auto`: an ACE 2 is any CH34x tty (USB vendor `1a86`, by-id or sysfs); an ACE 1
  is an `*ANYCUBIC*` by-id link. There have to be at least `ace_index` of them.
- An explicit `serial_port`: that path has to exist.

Right after boot (host uptime under 2 minutes) it waits up to `missing_ace_wait` seconds for the
ACE to enumerate. After that it checks once.

## When it's missing

- The unit isn't added to AFC: no ACE lanes, no ACE hubs, no serial connect or retries.
- Its lanes don't register on the unit, hub or extruder, so AFC keeps the heads' standalone lanes.
- `AFC.var.unit` keeps the ACE lanes' saved state (spools, colours, maps) as it was, and it's used
  again once the ACE is back.
- The console says `ACE_1: no ACE found, running without it`. The ACE temperature sensor reads 0 C.

Plug the ACE in and restart Klipper (`RESTART` or `FIRMWARE_RESTART`) to use it again.

## Options

```
[AFC_ACE2 ACE_1]
fallback_without_ace: True    # False: always use the ACE config (errors when it's missing)
missing_ace_wait: 20          # seconds to wait for it after a boot
serial_log: False             # True: log every ACE request and reply to AFC_ACE_serial.log
```

`serial_log` is off by default: the ACE is polled several times a second, so the log grows by about 10 MB every
few hours and costs Klipper CPU time. Turn it on only to debug ACE communication.
