# Creator 5 firmware updates

`scripts/build_c5_update.py` is the update maker for the Creator 5 and Creator 5 Pro. One run builds
Kalico for all four boards (mainboard, eBoard, heaterBoard, levelBoard), checks each build, and writes
everything a release needs from that one build:

- **Stock-style update packages**, one per printer model: FlashForge's encrypted USB update format
  with the stock MCU images replaced by these builds, each with its `.sha256` and `.manifest.json`.
  The OpenCreator installer flashes them (`MCU_PACKAGE=... make usb`), and tunneled printers install
  them from USB the stock way.
- **`live/`**: the four `.hex` files exactly as packaged, their `SHA256SUMS`, and `VERSION` (the
  `mcu_version` the boards will report). This goes in opencreator-fs `mcu-firmware/`, where the SoC
  updater flashes it live.

## Making a release

Build from a clean, committed checkout: the version the boards report and the manifest's
`repository.commit` come from it. The installer recognises boards that are already up to date by the
first 8 characters of that commit, so the version has to be a plain hash of at least 8 characters
(a checkout with release tags reports `v2026...-g<hash>`; clone without tags and set
`git config core.abbrev 9`).

The stock packages need FlashForge's update password in `C5_UPDATE_PASSPHRASE`, and a stock update
of each model as the template.

```sh
export C5_UPDATE_PASSPHRASE='...'
python3 scripts/build_c5_update.py all \
    --board eBoard --board heaterBoard --board levelBoard --board mainBoardGD \
    --template Creator5-1.9.9-1.3.0-20260907.gz \
    --template Creator5Pro-1.9.9-1.3.0-20260907.gz \
    --output-dir out/c5 \
    --output Creator5-kalico-all-mcus-<commit>-<date>.tgz \
    --output Creator5Pro-kalico-all-mcus-<commit>-<date>.tgz \
    --jobs 8
```

`--template` and `--output` pair up in order, one per model. `--output-dir` must be empty; it gets
`build/` (one folder per board) and `live/`.

The other commands do one stage each: `build --board` (one board), `validate`, `package` (from
existing builds) and `inspect` (look inside a package, `--decrypt` to see its contents).
