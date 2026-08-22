# GWM / Haval — openpilot port notes

Community port work for **GWM (Great Wall)** vehicles, with focus on the **2026 Haval H6 PHEV (MK4)** for openpilot on **comma four**.

| Item | Value |
|------|--------|
| **Primary target** | `GWM_HAVAL_H6_MK4` (Haval H6 PHEV ~2024–2026) |
| **Branch (opendbc)** | `mk4-haval-phev-2026` |
| **Fork** | [elmarculino/opendbc](https://github.com/elmarculino/opendbc) (based on nGoline / community GWM work) |
| **openpilot fork** | [elmarculino/openpilot](https://github.com/elmarculino/openpilot) branch `mk4-haval-phev-2026` (submodule → this opendbc) |
| **Device** | comma four · openpilot ~0.11.1 |
| **Last major doc update** | 2026-07-23 |

---

## Support level (MK4)

| Capability | Status | Notes |
|------------|--------|--------|
| Fingerprint / car ID | **Working** | Force fingerprint + CAN fingerprints; FW optional |
| Lateral (angle) | **Working** | `SteerControlType.angle`, panda `ANGLE_CONTROL` |
| Longitudinal (alpha) | **Working** | `openpilotLongitudinalControl`, `pcmCruise=False` (OP_CRUISE) |
| Engage | **Working** | Gear stalk **DOWN** (gentle), gated on gear D + motion |
| Set-speed (OP) | **Working** | Wheel scroll ±5 km/h (synthetic long-press) |
| Set-speed on OEM cluster | **In progress** | Re-TX ACC `0x2AB` with latched set + `CRUISE_STATE` activated |
| Regen on brake (PHEV) | **Working** | Light hysteresis on regen enable bits only |
| Safety tests | **Passing** | GWM suite (52 passed, 4 skipped last full run) |

**Not stock comma-supported.** Treat as alpha community port: always supervise.

---

## Architecture (MK4)

```
Engage:     GEAR_STALK STALK_DOWN → main_on / cruise available + panda OP_CRUISE arm
Lateral:    STEER_CMD angle (14-bit) @ main; EPS keepalive 0x147 @ camera
Long:       ACC_CMD (0x143) gas/brake + BRAKE_GAS_STATE for PHEV regen
Cluster:    re-TX ACC (0x2AB) @ main with ACC_SPEED_SELECTION + CRUISE activated
HUD lat:    LATERAL_STATE (0x23D) LKAS when latActive
Distance:   wheel follow buttons → gapAdjustCruise → openpilot personality (not OEM bars)
```

| Flag | Meaning |
|------|---------|
| `GwmSafetyFlags.OP_CRUISE` | Arm on gear stalk down, not MK3 FURTHER_DOWN-only path |
| `GwmSafetyFlags.ANGLE_CONTROL` | Angle command checks (VM) |
| `GwmSafetyFlags.LONG_CONTROL` | Longitudinal TX allowed |

---

## Recent updates (2026-07)

### Lateral
- `steerActuatorDelay` **0.15 → 0.20 → 0.28 s** + speed-scaled angle rate (0.6@≤25 → 1.0@≥45 kph) for low-speed hunt (routes 70/72/73).
- `MAX_ANGLE_RATE` **2.0 → 1.0** °/20 ms (100 → **50 °/s** low-speed backstop).  
  - *Note:* Tesla uses **5.0** as comfort/fault backstop; **1.0 is more damped**. If tight low-speed turns feel soft, raise rate toward **1.5–2.0** *before* changing delay (isolate knobs).
- `steerTempUnavailable` gated on real wheel divergence + EPS not obeying (not raw EPS fault bit).
- `MK4_ANGLE_ERROR_MAX` / not-obeying clip to reduce command windup vs EPS lag.
- Hands-on keepalive (spoofed torque on 0x147 → camera) to avoid OEM hands-off limp.
- Debounced lateral override (`OVERRIDE_TORQUE` 100 / instant 150) separate from shared `steeringPressed`.
- MK4 `steeringPressed` **hysteresis** ON **155** / OFF **120** (was 140/100) — hands-off EPS peaks still hit ~140 in curves (routes 70/72/73).
- MK4 `cruiseState.available` = in **Drive** and not acc-faulted (not only after stalk). Brake no longer clears main_on (cancel only) — cuts wrongCarMode flood.

### Longitudinal / cruise (openpilot + opendbc)
- OP_CRUISE: set-speed owned by openpilot (`pcmCruise=False`).
- Engage set-speed: **max(vEgo, 20 kph)** for GWM always (not stock 40 chill / **105 experimental** floor).
- Preserve set-speed across `available=False` cancel (non-pcm); never poison `v_cruise_kph_last` with 255.
- Stricter engage init so `vCruise` does not stay **255**.
- Regen: light hysteresis **ON ≤ −0.08**, **OFF ≥ +0.05** m/s² on **BRAKE_GAS_STATE only** (does not hold `BRAKE_OR_GAS_REQ` — that faulted OEM ACC).
- Removed dead `regen_level` / unused `REGEN_CONFIG` read.

### OEM cluster set-speed / Vmax + ICC icons
- Re-TX camera ACC **0x2AB** @ **10 Hz** (match OEM rate) with:
  - `ACC_SPEED_SELECTION` = openpilot set-speed (latched while engaged).
  - While OP enabled, force **stock activated chrome** on the frame (not only the state nibble):
    - `b18 = 0x1a` (CRUISE_STATE_2=activated **plus** constant `0x02` seen on every OEM eng frame)
    - `b21` low 3 bits = follow dashes **1..4** (never 0 = OEM “disabled” / no ICC)
    - `b21 |= 0x20` (bit set on all stock ACC-UI frames)
    - `b17 |= 0x80` (common on stock activated frames for Vmax chrome)
  - Follow dashes latched from personality; default **3** if HUD bars not ready yet.
  - Fallback set from vEgo (floor 20) if first-engage still unset for a frame.
- Panda safety: **0x2AB** on long TX list with `check_relay` (firmware rebuild + flash required once).
- **Packer zeroing (2026-07-29):** `ACC_CMD`/`LATERAL_STATE` packer rebuilds wiped camera bytes 0–7 and 32–63 / most of HUD (route 20 eng). MK4 now re-TX **raw camera shell** for ACC_CMD (overlay control block 8–31) and HUD (patch LKAS only). Re-test cluster icons after deploy.

### Fingerprint / bring-up
- `FINGERPRINTS` CAN map + force fingerprint / skip FW query in openpilot launch env for reliable ID after wipe.
- Calibration validated on-road (yaw ~0.3° after mount alignment).

### openpilot-side (fork)
- Branch `mk4-haval-phev-2026` with opendbc submodule bumps matching the above.
- Device deploy verified through commit `01c8c74` (opendbc `6dac3945`) for cluster icon work.

---

## Tuning snapshot (MK4)

| Parameter | Value | Role |
|-----------|--------|------|
| `steerActuatorDelay` | 0.28 s | Planner lag estimate (was 0.20; hunt damp) |
| `MAX_ANGLE_RATE` | 1.0 °/20 ms | High-speed ceiling |
| `MK4_ANGLE_RATE_*` | 0.6→1.0 @ 25–45 kph | Extra low-speed rate after VM limits |
| `MAX_LATERAL_ACCEL` | 3.0 m/s² | ISO-ish comfort |
| `MAX_LATERAL_JERK` | 2.5 m/s³ | Smoothing |
| `MK4_ANGLE_ERROR_MAX` | 4.0 ° | Cmd vs wheel windup limit |
| `MK4_REGEN_ON/OFF` | −0.08 / +0.05 m/s² | Regen enable hysteresis |
| Cruise init floor (GWM) | 20 kph | vs stock helper 40 |

**Compared to official angle cars:** delay is in-family (Ford ~0.2); angle rate backstop is **stricter** than Tesla (5.0). Architecture (OP-long, HUD set-speed, hysteresis) matches stock port patterns.

---

## Pending

| Item | Priority | Notes |
|------|----------|--------|
| **Cluster Vmax + ICC icons** | Fixed (code) | Root cause (route 57, 4137 frames): ACC_CMD b23 counter DEAD — 64-bit `BYPASSME_2` (b16–23, CRC@16+counter@23) truncated by float64 in the parser. Fix: preserve raw b16–23. Road-validate. |
| **Lateral “soquinhos” in light curves** | Fixed (code) | Root cause: carcontroller override latch false-tripping on hands-off EPS reaction torque (routes 56/57: 21 trips, tq 100–134, press=False). Latch now 130/170/10 (route 1f sim: 0 false, real grabs kept). Road-validate. |
| **Engage without steering feel** | Understood | The “latActive False 14%” windows are all ~1 kph stops: standstill gate with `steerAtStandstill=False`. Expected behavior, not a defect. |
| **OEM dual-beep on brake cancel** | Fixed (code) | Beep = 2 cluster transitions (`0x1a→0x0a→0x12`). Old demote never hit the wire (only patched while enabled). New: ~2 s post-cancel grace masks `0x0a`, lands 0x12 direct. Road-validate. |
| **Engage set 105 kph** | Done (code) | Experimental mode used `V_CRUISE_INITIAL_EXPERIMENTAL_MODE=105` as floor; GWM now always uses vEgo/20. |
| **Low-speed steer hunt** | In test | Delay 0.28 + rate 0.6→1.0 @25–45 kph (routes 70/72/73). Validate crawl + parking turns. |
| **Highway / >60 km/h validation** | Medium | Limited urban-only routes so far (max ~50 km/h in analyzed drives). |
| **Fingerprint without force env** | Low | Prefer pure CAN/FW ID after more routes. |
| **Upstream / nGoline PR** | Low | Branch is ahead of nGoline; not merged to commaai. |
| **OEM Haval voice on gas** | N/A (car) | Native ADAS TTS; mute in **car** settings, not openpilot. |

---

## Potential improvements

### Lateral
1. Raise `MAX_ANGLE_RATE` **1.0 → 1.5 → 2.0** if low-speed corners feel rate-limited (one knob at a time).
2. Optional further `steerActuatorDelay` tweak only after rate is settled.
3. Review `MK4_ANGLE_ERROR_MAX` (4°) if EPS tracking improves.
4. Reduce unnecessary `steerOverride` if drivers rest a hand (threshold narrative already documented in code).

### Longitudinal / cluster
1. If Vmax icon still intermittent: reverse-engineer remaining ACC/HUD chrome bits (`BYPASSME_CRUISE` / other 0x2AB fields) beyond `CRUISE_STATE_2`.
2. A/B: disable 0x2AB re-TX once to compare OEM frozen chrome vs OP path.
3. Optional Honda-style hysteresis on brake **command** if long still chatters (regen hyst already in place).
4. Gas+SET to raise set to vEgo already works in stock helper; document for drivers.

### Product / UX
1. Experimental Mode for **traffic lights / stop signs** (e2e long); requires OP long (already on).
2. Lane change: stock needs **blinker + ~≥32 km/h** — no extra toggle.
3. sunnypilot-style camera offset / MADS only if forking UI (not in this branch).

### Port hygiene
1. Keep safety tests green on every TX/safety change; reflash panda when `gwm.h` TX list changes.
2. Do not thrash dual lat knobs in one commit without road A/B.
3. Ignore local `_routes/` logs in git (analysis only).

---

## Driver quick reference

| Action | Control |
|--------|---------|
| Engage | Gear stalk **DOWN** (in D, moving) |
| Cancel | Cancel / brake (clears available) |
| Set-speed ± | Wheel scroll (±5 km/h) |
| Personality / follow style | Distance buttons → **relaxed / standard / aggressive** |
| Gas pedal | Longitudinal override (OEM may still announce) |
| Lane change | Blinker + speed ≳ 32 km/h |
| Traffic lights | **Experimental Mode** ON + OP long |

---

## Key files

| Path | Role |
|------|------|
| `opendbc/car/gwm/interface.py` | Params, delay, OP_CRUISE / angle flags, steerTemp logic |
| `opendbc/car/gwm/carstate.py` | Buttons, cruise available, angle, gas |
| `opendbc/car/gwm/carcontroller.py` | Angle cmd, long, regen hyst, ACC cluster re-TX |
| `opendbc/car/gwm/gwmcan.py` | CAN builders (`create_acc_cluster_mk4`, steer, long, HUD) |
| `opendbc/car/gwm/values.py` | Angle limits, error max, flags |
| `opendbc/car/gwm/fingerprints.py` | CAN + FW fingerprints |
| `opendbc/safety/modes/gwm.h` | Panda safety / TX list (0x2AB long) |
| openpilot `selfdrive/car/cruise.py` | GWM init floor 20, preserve set, no 255 on engage |

---

## Related openpilot settings

| Goal | Setting |
|------|---------|
| Stop for red lights / stop signs | **Experimental Mode** (needs OP long) |
| Lane change | Engaged + **turn signal** + ≳20 mph — no extra toggle |
| Gas does not fully disengage | **Disengage on Accelerator** = OFF |

---

## Safety / legal

This is experimental software. The driver is always responsible. Do not rely on alpha longitudinal or experimental mode in dense urban traffic without readiness to take over immediately.
