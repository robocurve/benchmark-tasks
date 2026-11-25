# Research r2: YAM motor duty/thermal limits, WR484080LSL keyholes, #8 pan-head screws

Compiled 2026-11-23 (all fetches on that date unless stated). Complete.
Conventions: "fetched" = page/PDF downloaded with curl/WebFetch on the given date and the quote copied from it;
"snippet" = web-search result summary only, wording and numbers unverified.

## Summary (2026-11-23)

1. **DM-J4340 ratings, resolved (HIGH, maker's own site + manuals):** 9 N m rated / 27 N m peak = original DM-J4340-2EC;
   12 N m / 40 N m = DM-J4340-2EC V1.1. Both identical at 24 V and 48 V. "7 N m" is not in any Damiao source.
   DM-J4310: 3 / 7 N m = V1.1; 3.5 / 12.5 N m = V1.2. Rated phase current: J4340 3 A (24 V, orig.) / 4.11 A (V1.1); J4310 2.5 A (V1.1).
2. **YAM supply = 24 V (HIGH, doc.i2rt.com).** I2RT never says which Damiao version is fitted (LOW: weak hints for V1.1);
   yam_arena's 27 / 7 N m effort limits = the conservative original-J4340 / V1.1-J4310 peak ratings.
3. **Temperature limits (HIGH, Damiao manuals):** driver over-temp "shutdown at 120° C"; motor (coil) over-temp
   "configurable, recommended ≤ 100° C"; register OT_Value range [80, 200). I2RT bench note: a DM4340 had OT_Value 100 °C
   (runtime) / 115 °C (Flash). No insulation class and no numeric max winding temperature are published.
4. **Duty (MEDIUM, Damiao test plots):** no duty rating or peak-hold time exists. Damiao's 24 V temperature-rise plots
   show the winding reaching ~99 °C (≈ the 100 °C trip) in ~9 min at 12 N m/36 rpm (J4340 V1.1) and ~5 min at
   3.5 N m/120 rpm (J4310 V1.2): "rated" torque is not a comfortable indefinite continuous rating.
5. **I2RT on payload/thermal (HIGH):** 2 kg nominal payload (YAM), 3 kg (Pro); docs give no thermal/duty guidance;
   400 ms CAN timeout; per-joint gravity_comp_factor [1.0,1.1,1.1,1.2,1.0,1.0]; TORQUE_MAX 28/10 are CAN scaling, not ratings.
6. **WR484080LSL keyholes, layout (HIGH, maker drawing x2 + HD product photo of the back):** 4 keyholes, 2 near each end,
   one each side of the centre line; double-ended slots (round centre, narrow slot both ways) running ALONG the length,
   same orientation at both ends; fixture slides lengthwise to lock.
7. **WR484080LSL keyholes, position (MEDIUM-LOW, scaled from drawing + photo):** ≈ 32.5 in apart lengthwise
   (≈ 7.7 in from each end) x ≈ 7.5 in apart across (≈ ±3.75 in from centre line).
8. **Keyhole size (LOW, scaled):** ≈ 1.0 in overall, centre hole ≈ 0.33 in, slot ≈ 0.17 in (±0.05) -- consistent with
   3/8 in / 3/16 in practice. 2-light sibling WR484056LSL: 2 keyholes on the centre line, ≈ 35 in apart.
9. **#8 pan head (MEDIUM-HIGH, B18.6.1 transcriptions, two agree):** Phillips pan: head Ø 0.306-0.322 in, height
   0.105-0.115 in; slotted pan height 0.085-0.096 in; shank 0.164 in. The fixture's screw gauge is NOT stated by the maker.
10. **#8 keyhole practice (LOW):** no standard found; woodworking/hardware practice ≈ 3/8 in head hole, 3/16-13/64 in slot.

---

## Q1. Damiao DM-J4340 / DM-J4310 motor duty and temperature limits; I2RT YAM

### Q1a. Damiao's own product pages (damiaokeji.com) -- PRIMARY, fetched 2026-11-23

Damiao = 深圳市达妙科技有限公司 (Shenzhen Damiao Technology Co., Ltd.). Official site http://www.damiaokeji.com/ .
The spec tables are PNG images inside each product page; I downloaded them and read them visually
(the Chinese text is quoted exactly as it appears in the image; English glosses are mine).
Copies saved in the scratchpad, not here.

Product list page (text, fetched): https://www.damiaokeji.com/index.php?c=category&id=2 (page 1) and &page=2
- "DM-J4340-2EC V1.1 额定扭矩：12Nm 峰值扭矩：40Nm"  -> product page id=135
- "DM-J4340P-2EC V1.1 额定扭矩：12Nm 峰值扭矩：40Nm" -> id=134
- "DM-J4340-2EC 额定扭矩：9Nm 峰值扭矩：27Nm"       -> id=100  (original version, no "V1.1")
- "DM-J4340P-2EC 额定扭矩：9Nm 峰值扭矩：27Nm"      -> id=101
- "DM-J4310-2EC V1.1 额定扭矩：3Nm 峰值扭矩：7Nm"    -> id=84
- "DM-J4310-2EC V1.2 额定扭矩：3.5Nm 峰值扭矩：12.5Nm" -> id=125
- "DM-J4310P-2EC 额定扭矩：3.5Nm 峰值扭矩：12.5Nm"  -> id=109
(额定扭矩 = rated torque, 峰值扭矩 = peak torque.)

**DM-J4340-2EC (original) spec table** -- page https://www.damiaokeji.com/index.php?c=show&id=100 ,
image https://www.damiaokeji.com/uploadfile/ueditor/image/202609/17891170979c5b85.png (read visually):
| row (verbatim) | 24 V column "DM-J4340-2EC" | 48 V column "DM-J4340-2EC （48V）" |
|---|---|---|
| 额定电压 (rated voltage) | 24V | 48V |
| 额定相电流 (rated phase current) | 3A | 2.5A |
| 峰值相电流 (peak phase current) | 8A | 9A |
| 额定扭矩 (rated torque) | 9NM (both) | |
| 峰值扭矩 (peak torque) | 27NM (both) | |
| 额定转速 (rated speed) | 36rpm (both) | |
| 空载最大转速 (no-load max speed) | 52.5rpm | 100rpm |
| 减速比 (gear ratio) | 40: 1 | |
| 相电感 / 相电阻 | 360uH / 0.88Ω | |
| 外径 / 高度 / 电机重量 | 57mm / 53.3mm / 约 362g | |
| 编码器位数 | 14 位 (2 encoders, magnetic single-turn) | |
Protection rows (verbatim):
- "驱动过温防护，防护温度：120°C，过温电机将退出“使能模式”。" (driver over-temperature protection, protection temperature 120 °C; on over-temperature the motor exits "enable mode")
- "电机过温防护，根据使用需求设定，建议不超过 100°C，过温电机将退出“使能模式”。" (motor over-temperature protection, set according to need, recommended not to exceed 100 °C; motor exits enable mode)
- 24 V: "电机过压防护，根据使用需求设定，建议不超过 32V，过压将退出“使能模式”。"; 48 V: "...建议不超过 52V..."
- "通讯丢失防护，设定周期内没有收到 CAN 指令将自动退出“使能模式”。" (CAN-timeout protection)
- "电机过流防护，根据使用需求设定，建议不超过 0.98，过流将退出“使能模式”。" (over-current, set as a fraction, recommended <= 0.98)
- 24 V: "电机欠压防护，若电源电压低于设定值，则退出“使能模式”，电源电压不低于 15V。"; 48 V: "...建议电源电压不低于 20V。"

**DM-J4340-2EC V1.1 spec table** -- page https://www.damiaokeji.com/index.php?c=show&id=135 ,
image https://www.damiaokeji.com/uploadfile/ueditor/image/202609/178911855682b790.png (read visually):
| row | "DM-J4340-2EC V1.1" (24 V) | "DM-J4340-2EC V1.1（48V）" |
|---|---|---|
| 额定电压 | 24V | 48V |
| 额定相电流 | 4.11A/3.1A | 4.11A/1.8A |
| 峰值相电流 | 19.85A/16.00A | 19.85A/11.2A |
| 额定扭矩 | 12NM (both) | |
| 峰值扭矩 | 40NM (both) | |
| 额定转速 | 36rpm | |
| 空载最大转速 | 56rpm | 112rpm |
| 减速比 | 40: 1 | |
| 相电感 / 相电阻 | 290uH (@25°C ) / 0.72Ω(@25°C ) | |
| 外径 / 高度 / 重量 | 57mm / 53.3mm / 约 362g | |
| 编码器位数 | 16 位 | |
(The "x/y" current pairs follow the V1.2 J4310 table's header "额定相 / 电源电流" = phase current / supply current.)
Protection rows: identical wording to the original (driver 120 °C; motor "建议不超过 100°C"; over-voltage 32 V / 60 V;
under-voltage 15 V for 24 V; over-current 0.98).

**DM-J4310-2EC V1.1 spec table** -- page https://www.damiaokeji.com/index.php?c=show&id=84 ,
image https://www.damiaokeji.com/uploadfile/ueditor/image/202609/17893807119fb81d.png :
- 额定电压 24V / 48V; 额定相电流 2.5A; 峰值相电流 7.5A; 额定扭矩 3NM; 峰值扭矩 7NM; 额定转速 120rpm;
  空载最大转速 200rpm / 400rpm; 减速比 10: 1; 相电感 340uH; 相电阻 0.65Ω; 槽数 24; 外径 57mm; 高度 46mm; 约 306g; 14 位.
- Protection: "驱动过温防护，防护温度：120°C..."; "电机过温防护，根据使用需求设定，建议不超过 100°C..."; over-voltage 32 V / 60 V;
  under-voltage 15 V / 20 V; over-current 0.98.

**DM-J4310-2EC V1.2 spec table** -- page https://www.damiaokeji.com/index.php?c=show&id=125 ,
image https://www.damiaokeji.com/uploadfile/ueditor/image/202609/1789380777907481.png :
- "额定相 / 电源电流 4.9A/3.1A@24V | 4.8A/1.6A@48V"; "峰值相 / 电源电流 20A/16.5A@24V | 20A/12.8A@48V";
  额定扭矩 3.5NM; 峰值扭矩 12.5NM; 额定转速 120rpm; 空载最大转速 200rpm / 450rpm; 减速比 10: 1; 约 320uH; 约 580mΩ; 16 位.
- Same protection rows (driver 120 °C; motor recommended <= 100 °C).

**What the Damiao tables do NOT give:** no insulation class, no maximum winding temperature other than the
"recommended not above 100 °C" motor over-temperature setting, no duty cycle, no time limit for peak torque, and no
torque-vs-time or thermal curve. Rated torque/current are listed identically at 24 V and 48 V (voltage only changes
no-load speed and supply current), so the rated/peak torque figures apply at either supply voltage.

**Resolution of the 7 / 9 / 12 N m DM-J4340 conflict (from the above):**
- 9 N m rated / 27 N m peak = the ORIGINAL DM-J4340-2EC (and J4340P), same at 24 V and 48 V.
- 12 N m rated / 40 N m peak = the newer DM-J4340-2EC V1.1 (and J4340P V1.1), same at 24 V and 48 V.
- "about 7 N m rated" (24 V listing seen in a snippet) appears in NO Damiao table; probably a seller error or a
  confusion with the DM-J4310 V1.1 peak (7 N m). Not supported by the maker.
- For DM-J4310: 3 / 7 N m = V1.1; 3.5 / 12.5 N m = V1.2 (and J4310P).

### Q1b. Damiao English user manuals (official, Damiao's Gitee repos) -- fetched 2026-11-23

Source: Damiao's official repo index https://gitee.com/kit-miao/damiao (".gitmodules" points to per-motor repos)
- https://gitee.com/kit-miao/DM-J4340-2EC  (folder 说明书/ = manuals, 测试数据/性能曲线/ = performance/test curves)
- https://gitee.com/kit-miao/DM-J4310-2EC
(Raw PDF links return HTTP 403 to curl; I got them with a sparse `git clone` of the public repos.)

**"DM-J4340-2EC  Geared Motor User Manual V1.1 2026-09-08.pdf"** (original 9/27 N m model), text layer:
- "Please operate the motor within the specifications listed below."
- "Model DM-J4340-2EC DM-J4340-2EC （48V）" / "Rated Voltage 24V 48V" / "Rated Phase Current 3A 2.5A" /
  "Peak Phase Current 8A 9A" / "Rated Torque 9NM" / "Peak Torque 27NM" / "Rated Speed 36rpm" /
  "Maximum No-Load Speed 52.5rpm 100rpm" / "Reduction Ratio 40: 1"
- "Drive over-temperature protection: shutdown at 120° C; the motor exits Enable Mode when triggered."
- "Motor over-temperature protection (configurable, recommended ≤ 100° C); the motor exits Enable Mode when triggered."
- "Safety Precautions 1. Operate the motor strictly within the specified environmental conditions and maximum winding
  temperature limits. Failure to do so may result in permanent damage."
- "5. ... High torque operation may generate heat. Avoid contact to prevent burns."
- "5. Features dual temperature protection mechanisms."
- "◈ Operating Voltage The operating voltage range for the 24V motor driver is 24V–28V, with a minimum operating voltage of
  20V and a maximum of 28V." / "For the 48V motor driver, the operating voltage range is 24V–48V, with a minimum operating
  voltage of 20V and a maximum of 58V."
- "The maximum phase current percentage can be limited by setting a percentage in the configuration tool. The default value
  is 0.8 (i.e., 80% of the measurable maximum current). It is recommended that this value not exceed 98%."
- "The torque constant of the motor can be considered constant within its rated range."
- "Torque–Speed (T–N) Curve The following curve shows the measured performance at 24 V, a constant speed of 36 rpm, and an
  ambient temperature of 25° C:" (curve is an image)
- Error codes: "B——MOS over-temperature;" "C——Motor winding over-temperature;" "E——Overload;"
- "T_MOS: Average MOSFET temperature on the driver (℃ )" / "T_Rotor: Average motor coil temperature (℃ )"
- Register table: "0x02 2 OT_Value Over-temperature Threshold RW [80.0,200) float"

**"DM-J4340-2EC  V1.1 Geared Motor User Manual V1.3 2026-08-13.pdf"** (V1.1, 12/40 N m):
- "Model DM-J4340-2EC V1.1 DM-J4340-2EC V1.1（48V）" / "Rated Phase Current / Rated Supply Current 4.11A/3.1A@24V 4.11A/1.8A@48V" /
  "Peak Phase Current/ Peak Supply Current 19.85A/16.0A@24V 19.85A/11.2A@48V" / "Rated Torque 12NM" / "Peak Torque 40NM" /
  "Rated Speed 36rpm" / "Maximum No-Load Speed 56rpm 112rpm"
- Same driver 120 °C and motor "recommended ≤ 100° C" wording; same 24 V driver range "24V–28V ... maximum of 28V".
- (Typo in the maker's manual: "Phase Resistance 0.72mΩ（@25℃）" -- the website table says 0.72Ω.)

**DM-J4310-2EC V1.1 manual V1.3 (2026-09-14)** and **V1.2 manual V1.4 (2026-09-14)**: same structure.
V1.1: "Rated Phase Current 2.5A" / "Peak Phase Current 7.5A" / "Rated Torque 3NM" / "Peak Torque 7NM" / "Rated Speed 120rpm".
V1.2: "Rated Torque 3.5NM" / "Peak Torque 12.5NM". Both: driver "shutdown at 120° C", motor "recommended ≤ 100° C",
24 V driver "24V–28V".

**Not in any Damiao manual (searched text layers for duration/second/minute/duty/insulation/class):** no peak-torque hold
time, no duty-cycle rating, no insulation class, no numeric "maximum winding temperature" beyond the ≤100 °C
recommended motor over-temperature trip. The manuals refer to "maximum winding temperature limits" without giving one.

### Q1c. Damiao temperature-rise (duty) curves -- the closest thing to a duty rating (fetched 2026-11-23, read visually)

All are PNG plots in Damiao's repos, path 测试数据/性能曲线/ (= "test data/performance curves"):
1. https://gitee.com/kit-miao/DM-J4340-2EC/raw/master/测试数据/性能曲线/V1.1/温升.png (温升 = temperature rise)
   Title "Motor Performance Curve At 24V DM4340 36rpm-12Nm"; y "motor_temp(℃)", x "time(s)".
   Read from plot: ~49 °C at t≈20 s, ~70 °C at ~110 s, ~80 °C at ~200 s, ~90 °C at ~370 s, ~95 °C at ~440 s,
   reaches ~99 °C at ~540 s and is flat at ~99 °C to the end (~590 s). Ambient/start temperature not stated.
2. .../V1.1/温升14Nm.png, title "Motor Performance Curve At 24V DM4340 36rpm-14Nm" -- the plotted trace is pixel-for-pixel
   the same as the 12 Nm plot (looks like a copy with a different title), so treat it as unreliable.
3. https://gitee.com/kit-miao/DM-J4310-2EC/raw/master/测试数据/性能曲线/V1.2/温升_3.5Nm.png
   "Motor Performance Curve At 24V DM4310 120rpm-3.5Nm": ~32 °C at 10 s, ~70 °C at 100 s, ~88 °C at 200 s,
   ~99 °C at ~310 s, flat at 99 °C to ~330 s.
4. .../DM-J4310-2EC/.../V1.2/温升.png "Motor Performance Curve At 24V DM4310 120rpm-3.2Nm": ~35 °C at 10 s, ~79 °C at
   200 s, ~91 °C at 370-400 s, ~99 °C at ~600 s, flat to 700 s.
5. V1.0 (original) J4340: only torque-speed/efficiency curves, no temperature curve:
   .../DM-J4340-2EC/raw/master/测试数据/性能曲线/V1.0/24V 36RPM 4340电机性能曲线图.png , title "4340 Motor Performance
   Curve At 24V 36 RPM": speed holds ~36 rpm to ~14 Nm and falls to ~28 rpm at 18 Nm (end of test); efficiency peaks
   ~64 % around 6-8 Nm; input power ~60 W at 10 Nm. No time axis, so no duty information.
   No temperature-rise curve exists in the repo for the DM-J4310 V1.1.

**Interpretation (mine, not the maker's):** at its rated torque and rated speed on 24 V, Damiao's own test shows the
winding sensor reaching ~99 °C -- i.e., at the recommended 100 °C motor trip -- within ~5 min (J4310 V1.2, 3.5 N m) to
~9 min (J4340 V1.1, 12 N m), then levelling off. So "rated torque" on these motors is NOT a comfortable indefinite
continuous rating: sustained rated torque sits right at the 100 °C trip. Plots end near 99 °C (perhaps the trip or a
plateau); they do not say whether the motor stopped. Stall (holding) heats the windings similarly for the same current
(copper loss ~ I^2 R), but concentrates it in whichever phase carries the most current; Damiao gives no stall rating.

### Q1d. I2RT (YAM maker) evidence -- fetched 2026-11-23

**doc.i2rt.com/products/yam.html** (YAM Arm Series page), live, fetched 2026-11-23:
- "Connect the 24 V power supply to the arm's power input"  -> the YAM runs on 24 V, so the 24 V Damiao columns apply
  (torque ratings are the same at 24 V and 48 V anyway).
- "Motor Series DM series brushless" / "Control Modes Joint position PD · Gravity compensation · Zero-gravity" /
  "Safety 400 ms motor timeout (configurable)" / "Mounting Table-top (standard)"
- Variants table: "YAM 3× DM4340 3× DM4310 [1.0, 1.1, 1.1, 1.2, 1.0, 1.0] Standard arm"; "YAM_PRO 3× DM4340 3× DM4310 ...
  Enhanced actuators"; "YAM_ULTRA 3× DM4340 3× DM4310 ... Top-spec build"
- "With temp_record_flag=True: # obs["temp_mos"] — (N,) MOS temperature per motor (°C) # obs["temp_rotor"] — (N,) rotor
  temperature per motor (°C)"
- Troubleshooting: "Joint sags slowly under gravity gravity_comp_factor too low Raise the per-joint factor by 0.05–0.10 increments"
- No payload number, no thermal limit, no duty statement on doc.i2rt.com.
**doc.i2rt.com/products/yam-standard** (live): "Shoulder motors 3× DM4340" / "Elbow / Wrist motors 3× DM4310" /
"Payload TBD — see i2rt.com" / "gravity_comp_factor:[1.0, 1.1, 1.1, 1.2, 1.0, 1.0]". It does NOT say which Damiao hardware
version (original vs V1.1 / V1.2) is fitted.

**github.com/i2rt-robotics/i2rt** (main @ ea116fe, 2026-11-23), files fetched by shallow clone:
- i2rt/robots/config/yam_v1.yml: motor_list 0x01-0x03 "DM4340", 0x04-0x06 "DM4310";
  "gravity_comp_factor: [1.0, 1.1, 1.1, 1.2, 1.0, 1.0]"
- i2rt/motor_drivers/utils.py: DM4340 "TORQUE_MAX=28", DM4310 "TORQUE_MAX=10". These are MIT-mode CAN *encoding ranges*
  (the motors' TMAX register), not ratings: dm_motor_registers.md says "| 23 | `TMAX` | float | rw | Max torque; the range
  used to encode torque in MIT mode (N·m). Must match `TORQUE_MAX` |". They equal Damiao's "Default TMAX" column
  (28 for DM-J4340 both versions, 10 for DM-J4310) in the 2026-10-28 Selection Guide.
- i2rt/motor_config_tool/dm_motor_registers.md: "| 2 | `OT_Value` | float | rw | Over-temperature protection threshold (°C) |"
  and an I2RT bench note: "Observed on a DM4340 (`sw_ver` 925970741): ... ten unrelated registers reverting from their
  runtime values to what Flash held — `UV_Value` 15→12, `KT_Value` 0→0.0513, `OT_Value` 100→115, ..."
  -> on at least one YAM-family DM4340 the over-temperature trip was 100 °C at runtime and 115 °C in Flash.
- README "Safety timeout": "The factory default is a **400 ms timeout** — motors enter damping mode if no command is
  received within 400 ms." / "Without the timeout, a failed gravity-compensation loop can produce uncontrolled torque."
- examples/arm7_osc/README.md (7-DOF bench check): "MOS temperature 33–40 °C" at idle; sim_robot.py: "# Simulated motor
  temperatures (°C) — typical idle values for real hardware. _SIM_TEMP_MOS = 35.0 _SIM_TEMP_ROTOR = 40.0"
- GitHub issues search (repo:i2rt-robotics/i2rt temperature / overheat / hot): no issue about overheating.
**i2rt.com store, YAM** (Wayback 2026-10-22 snapshot, fetched again 2026-11-23): "Nominal Payload: 2 kg | Max. workspace
range: 750 mm | Arm weight (including gripper): 5.02 kg"; "What's in the package? Robotic Arm Power Supplier CAN-USB
Cable Table Clamps ..." No duty/thermal text. (YAM Pro: "Nominal Payload: 3 kg", from the earlier note.)

**Which Damiao version is in the YAM?** Not stated anywhere by I2RT. Weak hints only: I2RT's DM4340 VELOCITY_MAX = 10
matches the V1.1's "Default VMAX" 10 in Damiao's Selection Guide (the original's default is 8); I2RT bench-read
KT_Value 0.0513 on a DM4340 (not decodable to a version from public data). Arms bought in 2025-26 could have either.
Conservative choice for a benchmark: use the ORIGINAL ratings (J4340 9 N m rated / 27 N m peak; J4310 V1.1 3 / 7 N m),
which is what yam_arena already uses for effort limits (27 / 7).

**Damiao English Selection Guide** https://gitee.com/kit-miao/damiao/raw/master/2026-Damiao%20Technology-%20Selection%20Guide-2026-10-28.pdf
(fetched 2026-11-23; "Shenzhen Damiao Technology Co., Ltd. Motor Module Selection Guide"), rows (columns: Rated Torque (Nm),
Peak Torque (Nm), No-load Speed, Rated Speed, Gear Ratio, Dimensions, Weight, Recommended Supply Voltage, Operating Voltage
Range, Rated Phase Current, Peak Phase Current, Rated Power (W), ... Default PMAX, VMAX, TMAX):
- "DM-J4340-2EC 9 27 52.5 36 40 57*53.3 ≈362 24 15~32 3 8 33.9 ..." (TMAX 28)
- "DM-J4340-2EC V1.1 [12 40] 56 36 40 57*53.3 24 15~32 4.11 19.85 45.2 ..." (TMAX 28)
- "DM-J4310-2EC V1.1 3 7 200 120 10 57*46 ≈306 24 15~32 2.5 7.5 37.7 ..." (TMAX 10)
- "DM-J4310-2EC V1.2 3.5 12.5 200 120 10 57*46 ≈306 24 15~32 4.9 20 44.0 ..."
  Rated power 33.9 W (J4340 orig.), 45.2 W (J4340 V1.1), 37.7 W (J4310 V1.1), 44.0 W (J4310 V1.2). No thermal columns.

### Q1e. Search snippets (not fetched) for completeness
- Sellers (openelab, aifitlab, mybotshop): DM-J4340 "9Nm rated torque, 27Nm peak torque" -- matches the original model.
- roboticscenter.ai "Damiao DM-J4340-2EC-48V": "12 N·m Rated Torque", "40 N·m Peak Torque", "V1.1" -- matches V1.1.
- No source anywhere gave a peak-torque hold time for these motors.


## Q2. WR484080LSL keyhole slots

### Q2a. Maker's manuals: housing-back drawings (fetched 2026-11-23, rendered to PNG and measured)

Sources (both on images.thdstatic.com, fetched 2026-11-23; same URLs as quotes_product.md §1f):
- OLD manual (Item #1005 578 285, Model #WR484080LSL / WR484056LSL), 8 pp, vector drawings:
  https://images.thdstatic.com/catalog/pdfImages/33/33f09c44-4c2e-4052-894e-84945253908f.pdf
- NEW 2023 manual (Item #1006 805 989 / 1005 578 285, Model # WR484056LSLV / WR484080LSLV), 8 pp:
  https://images.thdstatic.com/catalog/pdfImages/4d/4d7ab39c-d45d-4c5c-b61e-9463f3af85b7.pdf

Text (old manual p.4, step "2 Removing the knockout", text layer): "Identify the location where the fixture (A) will be
installed. Using a screwdriver, remove the appropriate knockout(s) (1) to properly position the fixture." / "Locate the
mounting holes (2) for the next step." / "If multiple fixtures are to be row mounted, use a screwdriver to remove the end
knockout (3)."  Step 4A (p.5): "Guide the heads of the four installed wood screws (AA) into the keyhole slots of the
fixture (A)." / "Slide the fixture (A) until the wood screws (AA) are secured into the narrow ends of the keyhole slots."

**What the drawings show (verified by viewing the renders):**
1. OLD manual p.4 isometric of the 4-light housing back: callout "2" (mounting holes) points to FOUR keyhole slots,
   TWO NEAR EACH END, one on each side of the long centre line (none on the centre line). Callout "1" = three round
   knockouts on the centre line (one at the middle, one near each end); "3" = end knockout.
2. NEW 2023 manual p.4 shows BOTH housings in one figure:
   - "4' 2 Light Square Lens": TWO keyholes ("2"), ONE NEAR EACH END, ON the long centre line.
   - "4' 4 Light Square Lens": FOUR keyholes, the same layout as the old manual (pixel-identical geometry).
3. Keyhole SHAPE: every keyhole is drawn as a DOUBLE-ENDED slot: a round large hole in the middle with a narrow slot
   running out on BOTH sides, the slot axis ALONG THE FIXTURE LENGTH. All keyholes have the same orientation (no
   rotated ones at the other end). The step-4A detail (old manual p.5) shows a Phillips pan-type screw head passing the
   round centre hole, then a bold arrow parallel to the fixture's long side, and the screw sitting in one narrow end
   ("AA"). So the fixture is slid lengthwise; since the slot is double-ended, either direction locks.

**Measurements from the vector drawing (my estimate; the drawing is not dimensioned):** rendered at 1200 dpi,
the long axis is horizontal and undistorted (oblique projection).
- Scale check: the outer end-wall lines are 3483-3489 px apart (= 47.96 in overall length -> ~72.7 px/in). On that
  scale the round knockouts measure ~0.93 in, close to a standard 7/8 in (0.875 in) knockout, so the drawing looks
  close to scale (about ±7 %).
- 4-light (WR484080LSL): keyhole centres 2346 px apart along the length at both rows -> **~32.3 in apart lengthwise**,
  i.e. **~7.8 in in from each end** of the 47.96 in fixture (531 px = 7.3 in from the inside of each end wall).
  Across the width the keyholes sit at 19.8 % and 80.2 % of the back panel's width, symmetric about the centre line
  (60.4 % of the panel width apart). The panel's true width is not given; from the drawing's foreshortening (circles
  drawn ~0.85-0.88 as tall as wide) it is ~12.5-13 in, giving **~7.5-8 in across (about ±3.8-3.9 in from the centre
  line)**; if the panel were the full 14.13 in it would be 8.5 in. So the four keyholes form roughly a
  **32 in x 7.5-8 in rectangle** centred on the fixture.
- Keyhole size (4-light): outline 76 px long -> **overall slot length ≈ 0.95-1.0 in**. At 4800 dpi: centre-hole
  diameter ≈ 0.33 in (≈ 8.5 mm), narrow-slot width ≈ 0.17 in (≈ 4.3 mm) after correcting the foreshortening. Line
  weight is ~0.07 in at this scale, so treat these as ±0.05 in. They are consistent with a #8 screw (head ~0.31 in,
  shank 0.164 in), but they are drawing measurements, not maker dimensions.
- 2-light sibling (WR484056LSL, from the 2023 manual): keyholes on the centre line, centres ~2538 px = **~34.9 in
  apart**, i.e. **~6.5 in from each end**; drawn slightly longer (~1.15 in overall). Same lengthwise orientation.

**Confidence:** layout (count, two-per-end off-centre for 4-light, lengthwise double-ended slots) = HIGH (maker's
drawing, two editions agree). Numeric positions = MEDIUM-LOW (scaled from an undimensioned drawing; ±0.5 in
lengthwise, ±1 in across). Hole sizes = LOW (±0.05 in or worse).

### Q2b. Home Depot product photo of the housing back (fetched 2026-11-23) -- REAL PRODUCT, confirms the drawing

Image list taken from the Wayback snapshot of the WR484080LSL product page (2025-09-01):
https://web.archive.org/web/20250901064514/https://www.homedepot.com/p/Commercial-Electric-1-Pack-4-ft-8000-Lumens-Integrated-LED-White-Wrap-Light-4000K-WR484080LSL/313512347
The gallery has 5 WR484080LSL images; one is a straight-on photo of the BACK of the housing:
https://images.thdstatic.com/productImages/c5c47f60-981b-4d11-8376-8c784fcf9afd/svn/white-commercial-electric-wraparound-lights-wr484080lsl-66_1000.jpg
(1000 x 1000 px; larger sizes return the same file). Fetched live from images.thdstatic.com 2026-11-23.

What it shows (viewed, then measured):
- FOUR keyholes, two near each end, one above and one below the long centre line; a round knockout at the centre
  and knockouts near each end on the centre line -- exactly the manual's layout.
- Every keyhole is a dark "-o-" shape: round centre with narrow slots on BOTH sides, the slot running ALONG THE
  LENGTH. Same orientation at both ends.
- Measured after a perspective correction (homography from the four outline corners; length taken as 47.96 in):
  keyhole centres 7.90 / 7.57 in from the left end and 7.40 / 7.50 in from the right end; lengthwise spacing
  32.7 in (upper pair) and 32.9 in (lower pair). Across the width they sit at 20 % and 76 % of the visible outline
  (the centre knockout lands at 45 %, so the outline includes some lip on one side); with the photo's ~13.3 in
  apparent width that is ~7.4 in between rows.
- Keyhole size in the photo (~19.5 px/in, half-maximum widths): overall length 17-19 px ≈ 0.9-1.0 in; centre hole
  5-6 px ≈ 0.26-0.31 in; slot 2-3 px ≈ 0.10-0.15 in. JPEG blur makes holes read small at this resolution.

**Combined best estimate for WR484080LSL (drawing + photo):**
- 4 keyholes; rectangle ≈ **32.5 in (±0.5) lengthwise x 7.5 in (±0.5) across**, centred on the fixture; centres
  ≈ **7.7 in (±0.4) from each end** and ≈ **3.75 in either side of the centre line**.
- Double-ended slots along the length, same at both ends; fixture slides lengthwise (either way) to lock.
- Slot overall ≈ 1.0 in long; centre (large) hole ≈ 0.3-0.35 in (≈ 8-9 mm); slot width ≈ 0.15-0.17 in (≈ 4 mm).
  Maker does not publish these numbers.
- Sibling WR484056LSL (2-light, 8.5 in wide): 2 keyholes on the centre line, ~6.5 in from each end, ~35 in apart
  (2023-manual drawing only; no back photo of the 2-light in the HD gallery).

Not done for lack of time: HD customer-review photos (reviews load from a separate API, not in the snapshot),
YouTube storyboards, HD Supply image gallery. The HD product video URL in the snapshot is a signed Brightcove link.


## Q3. #8 pan-head wood screw head; keyhole sizes for #8

### Q3a. The WR484080LSL's own screws: size NOT stated by the maker
Both manuals and the spec sheet (fetched 2026-11-23) call them only "Phillips head wood screws" ("AA ... 4"), with
"Using a 1/8 in. drill bit, drill pilot holes" and "leaving a 3/8 in. gap between the screw head and the mounting
surface". No gauge, length or head style is given. The step-4A drawing shows a domed Phillips head (pan-type).
So "#8 pan head" is an assumption; it fits the 1/8 in pilot (Aspen Fasteners' Table 4, below: #8 rolled-thread body
pilot "1/8") and the drawn keyhole size (Q2), but is not confirmed.

### Q3b. ASME B18.6.1-1981 (R2016) pan-head wood screw dimensions, No. 8 (fetched 2026-11-23)
Primary standard is paywalled; these are transcriptions. Two independent transcriptions agree on the head diameter.
1. globalfastener.com transcription of B18.6.1 Table 11 (Type I = Phillips cross-recessed pan head):
   https://www.globalfastener.com/standards/detail_4568.html  (page title "ASME/ANSI B 18.6.1 (T11) - 1981 (R2016)
   Type I Slotted Pan Head Wood Screws [Table 11]" -- the site's title says "Slotted" but Table 11 / "Type I" and its
   columns m (recess dia), Socket No., Penetration depth are the cross-recess table). Columns "d max min | d k max min |
   k max min | R max min | m max min | t max min | n min | Socket No. | Penetration depth max min | r max"; the 8# column:
   - TPI "15"; d (body) "0.168" / "0.157"
   - **d k head diameter max "0.322", min "0.306"**
   - **k head height max "0.115", min "0.105"**
   - R "0.052" / "0.015"; recess dia m "0.182" / "0.169"; Socket No. "2" (Phillips #2)
2. Same site, Table 10, slotted pan head: https://www.globalfastener.com/standards/detail_4567.html
   ("ASME/ANSI B 18.6.1 (T10) - 1981 (R2016) Slotted Pan Head Wood Screws [Table 10]"), 8#: d k "0.322" / "0.306",
   **k (height) "0.096" / "0.085"**, slot width n "0.054" / "0.045".
3. Machinery's Handbook 31st ed. p.520 (flippingbook viewer, read via WebFetch 2026-11-23; text layer scrambled):
   table title "ANSI Flat, Pan, and Oval Head Wood Screws ANSI/ASME B18.6.1-1981 (R2016)"; the size-8 head-diameter
   entry reads ".322 .306 .052" -- matches the pan-head diameter and head radius above. Height not readable.
   https://online.flippingbook.com/view/954046886/520
4. Basic diameter: Aspen Fasteners wood-screw spec sheet https://aspenfasteners.com/content/pdf/woodscrew_spec_sheet.pdf
   Table 1: "8 15 0.164 5/32 0.112 7/64" (nominal 8, TPI 15, shank 0.164 in, root 0.112 in). Table 4 (pilot holes):
   "8 5/32 1/8 5/64 3/32 3/8 2 2" (body cut 5/32, body rolled 1/8, softwood 5/64, hardwood 3/32, countersink 3/8,
   Phillips #2, Square #2). Its Table 3 gives only flat/oval "0.312" and ROUND "0.298" average head diameters for #8
   (no pan column).
   For comparison, CDE Fasteners round-head B18.6.1 table (https://cdefasteners.com/sites/default/files/product-specs/woodscrewroundhds.pdf):
   "8 .309 .287 .120 .107 ..." (round head dia max/min, height max/min).
**Answer:** #8 Phillips pan-head wood screw (B18.6.1): head Ø 0.306-0.322 in (7.8-8.2 mm), head height 0.105-0.115 in
(2.7-2.9 mm); slotted pan: same diameter, height 0.085-0.096 in. Shank (basic) 0.164 in (4.2 mm). Caveat: the
screws actually packed with the light are unbranded and could be washer-head or another gauge.

### Q3c. Typical keyhole sizes for #8 screws (practice; weak evidence)
- No standard (ASME/IES/UL) for keyhole slot size was found. Light-fixture makers do not publish keyhole dimensions
  (none of the Commercial Electric, NICOR, Lithonia sheets seen give them).
- Amana Tool #45650 "Carbide Tipped Keyhole 3/8 Dia x 3/8 x 1/4 Inch Shank" (Wayback 2019-10-13 snapshot of
  https://www.amanatool.com/45650-carbide-tipped-keyhole-3-8-dia-x-3-8-x-1-4-inch-shank.html , fetched 2026-11-23):
  "Diameter (D) 3/8 Cutting Height (B) 3/8 Shank (d) 1/4 Overall Length (L) 1-1/2 B1 3/16 Flutes 1 D1 13/64" --
  a woodworking keyhole: 3/8 in head pocket, 13/64 in neck slot. "Form keyhole slots in plaques, picture frames, and
  other wall-hanging items". (Does not name a screw gauge.)
- Lee Valley 75964 Keyhole Slot Plug (Wayback 2026-04-16, fetched 2026-11-23): "The keyhole opening is sized to fit
  common #8 screws." (no dimensions given).
- Search snippets, not fetched: Lee Valley 3/8 in keyhole bit "A: 3/8" B: 1/8" ..." (head 3/8, neck 1/8 -- neck is
  the bit's neck, not necessarily the slot); metric keyhole hanging plates "Keyhole size is 14mm x 7.5mm ... mounting
  hole of 3.5mm" and "13mm x 7mm ... 4mm".
- Engineering constraint (from Q3b numbers): large hole must exceed the head's 0.322 in max -> 3/8 in (0.375, 9.5 mm)
  is the common choice; the slot must exceed the 0.164-0.168 in shank and stay well under the 0.306 in min head ->
  3/16 in (0.1875, 4.8 mm) to 13/64 in (0.203 in, 5.2 mm) is typical; a 1/8 in neck would not pass a #8 shank.
- The WR484080LSL's drawn/photographed keyholes (Q2: centre ~0.33 in, slot ~0.17 in, ±0.05) are consistent with this
  3/8 in / 3/16 in practice within measurement error.

