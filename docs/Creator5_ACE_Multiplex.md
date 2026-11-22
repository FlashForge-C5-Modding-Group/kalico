# ACE multiplexing on a Creator 5

With a multiplexing adapter, several ACE lanes (from one or more ACEs) feed the
same Creator 5 head. A tool change between two lanes of one head swaps the
filament instead of only the head:

1. The head is picked up, if it is not already mounted.
2. It goes to the purge bucket, so everything that oozes lands there. Z never
   goes down during the swap: the purge runs at the current height
   (`C5_TOOL_PURGE KEEP_Z=1`).
3. The lane in the head is unloaded: the head pulls it out of the gears on its
   own first (`unload_head_first`, 50 mm, or half of `tool_stn_unload` if more;
   the ACE doesn't pull at all until then), then the rest of `tool_stn_unload`
   while the ACE winds it back in one move at full speed,
   stopping the moment the head's filament switch clears (read raw, not through
   its 10 s debounce), then `multiplex_park_distance` (100 mm) more, clear of the
   adapter.
4. The new lane is fed to the switch, on into the gears and to the nozzle.
5. The flush is purged into the bucket: the slicer's `PURGE_LENGTH`, else
   `multiplex_purge_length`, else the toolchanger's purge length.
6. The print carries on from where it stopped.

A change to a lane on another head that already holds that lane stays a plain
head swap. Nothing changes unless `multiplex` is on.

## Setup

Give each ACE unit its `ace_index`, turn `multiplex` on, and list in `heads`
which head (tool 0-3) each of its four slots feeds, `-` for an unused slot.
Several units can feed the same head:

```ini
# Two ACEs: slots 1-2 of each into T0's head, slot 3 into T1's, slot 4 unused
[AFC_ACE2 ACE_1]
serial_port: auto
ace_index: 1
mode: direct
multiplex: True
heads: 0, 0, 1, -

[AFC_ACE2 ACE_2]
serial_port: auto
ace_index: 2
mode: direct
multiplex: True
heads: 0, 0, 1, -
```

The unit makes a lane and a hub for every slot in `heads` (`ACE_2_lane1`,
`ACE_2_hub1`, ...). A slot that already has an `[AFC_lane]` in the config keeps
it, so existing lanes and their settings stay as they are.

**Tool numbers.** The heads keep T0-T3, so a head with no ACE lane still works
as its own tool. With multiplexing the first lane into a head (by `ace_index`,
then slot) takes that head's number, and every other lane counts up from T4 in
the same order. One ACE with all four slots into T0's head gives T0, T4, T5 and
T6. Without multiplexing a lane takes its head's number and a head takes only
one slot. Lanes written in the config keep their own `map`: give them numbers
the heads don't use.

| Option (ACE unit) | Default | |
| --- | --- | --- |
| `multiplex` | False | Swap filament on a change between lanes of one head, park unloaded lanes just past the head switch, and give every lane its own tool number. |
| `heads` | | The head (0-3) each slot feeds, `-` for unused. Makes the lanes for slots with no `[AFC_lane]` of their own. |
| `lane_dist_hub` | 60 | `dist_hub` of the lanes it makes. |
| `hub_bowden_length` | 900 | `afc_bowden_length` and `afc_unload_bowden_length` of the hubs it makes (park point to the head switch). |
| `multiplex_park_distance` | 100 | mm past the head's switch an unloaded lane waits. Must reach past the adapter's merge point. |
| `unload_head_first` | 50 | mm the head retracts by itself before the ACE pulls on an unload (any mode). |
| `multiplex_purge_length` | toolchanger's | Flush in mm when the tool change gives no `PURGE_LENGTH`. |

Each path through the adapter has its own length; measure it with
`AFC_ACE_CALIBRATE UNIT=<unit> LANE=<lane>`. For a lane the unit made, the
result goes to `AFC/AFC_auto_vars.cfg` (there is no section of its own to write
to): add `[include AFC/AFC_auto_vars.cfg]` to `printer.cfg` after
`printer.ace*.cfg` to keep it, or write the lane's own `[AFC_hub <unit>_hub<n>]`
with that `afc_bowden_length`. Options the unit sets and a section leaves out
are still filled in.

## Slicer

Each lane is a filament in the slicer, by its T number. Pass the flush the
slicer works out for each change, in the change-filament G-code:

```
T[next_extruder] PURGE_LENGTH=[flush_length]
```

## Notes

- A lane parked past the switch is remembered across restarts, so its next
  load only feeds the last stretch.
- If the switch never clears while winding back, the change stops (pausing the
  print) with the head still holding the filament: clear the path, then run
  `TOOL_UNLOAD LANE=<lane>` and resume.
