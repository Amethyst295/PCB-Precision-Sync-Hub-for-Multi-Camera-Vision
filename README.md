# Precision Synchronization Hub for Multi-Camera Vision

**AI-SED 2026 · Team Borrenautas · Tecnológico de Monterrey, Campus Sonora Norte**

A 4-layer PCBA that turns an IEEE 1588 (PTP) network time reference into isolated, hardware-timed trigger
pulses for up to 8 cameras, measures each camera's real exposure start (strobe), and uses AI both to place
the board and to keep it synchronized in operation. This repository holds the firmware, the AI models and
every script used to produce the results of the Phase 1 technical document.

| Item | Value |
|---|---|
| MCU | STM32F407VET6 (Cortex-M4F, 168 MHz, Ethernet MAC with IEEE 1588 timestamping) |
| PHY / clock | LAN8742A (RMII) · ECS-TXO-2016 25 MHz TCXO + SN74LVC2G34 buffer |
| Channels | 8 isolated trigger outputs + 8 isolated strobe inputs (ISO7740F ×4, 74AC541, RFM-0505S) |
| Power | 7–12 V in → LMR16006X buck (5.05 V) → TLV1117LV33 (3.3 V) |
| Board | 85 × 75 mm, 4 layers (AI-seeded placement) |

## Contents

| File | Purpose | Document section |
|---|---|---|
| `ptp_sim.py` | PTP link simulator, MLP bias-corrector training, export of `mlp_weights.h`, closed-loop PI vs AI evaluation | 4.2–4.4, 5.3, Annex D–E |
| `anomaly.py` | Camera and fault simulator, decision-tree training, export of `anomaly_tree.c`, threshold baseline | 4.2–4.4, 5.2, Annex D–E |
| `ga_place.py` | Placement model: parts, isolation zones, weighted nets, cost function, genetic algorithm | 4.3 A |
| `ga_final.py` | GA + simulated-annealing refinement + legalization → `placement_GA.csv`, figures 6 and 9 | 5, Annex E |
| `baseline.py` | Conventional block placement (90 × 90 mm) and before/after comparison → `placement_before.csv` | 5.1 |
| `calcs.py` | Skew budget, frequency error, ripple, power budget, IPC-2221 trace widths | 3.2–3.3 |
| `fig_block.py`, `figs_ml.py`, `fig7.py` | Figures 1, 3, 7 and 8 | — |
| `*_results.json`, `ga_result.json`, `*.csv` | Results used in the document | — |
| `firmware/STM32F407VE_1/` | Proteus VSM project sources, including the trained `mlp_weights.h` and `anomaly_tree.c` | 3.1, 3.2, 4.3 |
| `firmware/tools/` | Stand-alone GCC build files and instruction-level emulator for the Flash/RAM/inference figures | 4.4 |
| `docs/ERC.erc` | Proteus ERC report | 3.2, Annex A |
| `docs/BOM_for_quote.csv` | Line-by-line BOM with manufacturer part numbers, quantities and reference designators (ready for Digi-Key / Mouser BOM tools) | 6.1 |
| `docs/BOM_prices.xlsx` | Unit price of every component at 1 and 1,000 pieces (MXN) with its source, used for section 6.1 | 6.1 |

`placement_GA.csv` is the placement used on the final PCB (x, y in mm from the bottom-left board corner, rotation in degrees).

## Reproducing the results

```bash
pip install -r requirements.txt

python ptp_sim.py 80          # 320 sequences; writes mlp_weights.h + ptp_results.json (~7 min)
python anomaly.py 120         # 1,080 camera sequences; writes anomaly_tree.c + anomaly_results.json
cp mlp_weights.h anomaly_tree.c firmware/STM32F407VE_1/

python ga_final.py 85 75 11,22 800   # final placement on 85 × 75 mm (seeds 11 and 22, 800 generations)
python baseline.py                   # conventional placement + fig9_before_after.png
python calcs.py                      # writes calcs.json
python fig_block.py && python figs_ml.py && python fig7.py
```

All random generators are seeded, so the runs reproduce the numbers in the document exactly.

## Key results (simulation)

| Result | Value |
|---|---|
| Channel-to-channel skew | 0 timer cycles; ≤ 6.1 ns hardware worst case (calculated) |
| Proteus VSM | 8 outputs at 30.0 fps (33.1 ± 0.1 ms), coincident within the 79 µs display resolution |
| AI placement vs conventional | −21 % area, −30 % critical-net wirelength, −83 % clock-buffer → PHY path |
| PTP offset RMSE, heavy traffic | 5.19 µs (PI servo) → 4.79 µs (PI + MLP), −7.7 %; neutral (±0.3 %) in light/medium traffic |
| Fault classifier (8 classes) | 97.2 % accuracy, macro-F1 0.959 (tuned thresholds: 93.7 % / 0.916); detection 3 s median, 6 s p90 |
| Firmware footprint | ≈ 15 KB Flash, ≈ 8 KB static RAM; MLP ≈ 19 µs, tree ≈ 0.4 µs (emulated, -O2) |

The ML models are trained on synthetic data and must be re-validated with real network captures.

## Firmware notes

* Build with Proteus VSM Studio (target STM32F407VE). The CMSIS and device headers (`stm32f407xx.h`,
  `core_cm4.h`, `cmsis_gcc.h`, `system_stm32f4xx.h`, …) come from the VSM Studio project template or from
  STM32CubeF4 (`Drivers/CMSIS`) and are not stored here.
* VSM Studio defines `STM32F40_41xxx`, which selects the simulation build (`SIM_F407`): no Ethernet and a
  free-running 30 fps frame clock. In that build the network-health features are zero, so the fault monitor
  reports `PTP_LOSS` after the 60 s calibration, which is expected without a PTP master.
* PTP stack integration point (`main.c`): apply the servo output as `ptp_hw_adjust_ppb(-ppb)` (negative feedback).
* Size and inference measurement (from `firmware/tools/`, with the CMSIS headers in `<cmsis>`):

  ```bash
  arm-none-eabi-gcc -O2 -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T f407.ld \
    --specs=nano.specs --specs=nosys.specs -nostartfiles -I ../STM32F407VE_1 -I <cmsis> \
    startup.c bench.c ../STM32F407VE_1/system.c ../STM32F407VE_1/anomaly_tree.c ../STM32F407VE_1/ptp_servo.c -o bench.elf
  python emu.py bench.elf
  ```

## Team

Luis Pablo Granados Monge · Víctor Manuel Arévalo Urrutia · Diego de Silva Hernández · Adrián Madrid Castro
Advisor: Jesús Leopoldo Montijo Durán
