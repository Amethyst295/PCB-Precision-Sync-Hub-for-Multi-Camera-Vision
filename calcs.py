"""Engineering calculations for the Sync Hub document (values derived from the schematic + datasheets)."""
import numpy as np, json, math
R = {}
# ---------- 1. trigger timing ----------
FT = 84e6                                      # TIM2/3/4 kernel clock
R['tick_ns'] = 1e9 / FT
f = np.arange(1, 1001)
arr = np.round(FT / f)
err_ppm = np.abs(FT / arr - f) / f * 1e6
R['quant_err_ppm_max_1_1000Hz'] = float(err_ppm.max()); R['quant_err_worst_f'] = int(f[err_ppm.argmax()])
R['quant_err_30fps_ppm'] = float(abs(FT / round(FT / 30) - 30) / 30 * 1e6)
R['tcxo_ppm'] = 2.5                            # ECS-TXO-2016, -40..+85 C
R['freq_err_free_pct'] = (R['tcxo_ppm'] + R['quant_err_ppm_max_1_1000Hz']) * 1e-4
R['freq_err_30fps_free_pct'] = (R['tcxo_ppm'] + R['quant_err_30fps_ppm']) * 1e-4
R['pulse_max_us'] = 65535 / FT * 1e6           # TIM3/TIM4 are 16-bit (one-pulse ARR)
# ---------- 2. channel-to-channel skew budget (worst case) ----------
skew = {'TIM3/TIM4 start (same TRGO edge, same 84 MHz clock)': 0.0,
        'GPIO output stage (same port speed setting)': 0.5,
        'ISO7740 part-to-part skew tsk(pp), 3.3 V side1': 4.5,
        '74AC541 output-to-output (not specified; est.)': 1.0,
        'PCB trace mismatch 10 mm @ 7 ps/mm': 0.07}
R['skew_budget_ns'] = skew; R['skew_total_ns'] = sum(skew.values())
R['latency_ns'] = {'ISO7740 tPLH typ/max': (11, 16), '74AC541 tPD max': 6}
# ---------- 3. isolated-driver output edge into a camera cable ----------
Rout_drv = 25.0; Rser = 82.0
for Lc in (2, 5):
    C = Lc * 100e-12 + 15e-12 + 5e-12        # 100 pF/m cable + camera input + SRV05-4
    tau = (Rout_drv + Rser) * C
    R[f'rise_10_90_ns_{Lc}m'] = 2.197 * tau * 1e9
# ---------- 4. power stage ----------
VFB = 0.765; Rt, Rb = 56e3, 10e3
Vo = VFB * (1 + Rt / Rb); R['v5_out'] = Vo
L = 22e-6; fsw = 0.7e6; Cnom = 2 * 22e-6; Ceff = 2 * 10e-6; esr = 3e-3
for Vin in (7, 12):
    dI = Vo * (Vin - Vo) / (Vin * L * fsw)
    vr = dI / (8 * fsw * Ceff) + dI * esr / 2
    R[f'buck_dIL_A_{Vin}V'] = dI; R[f'buck_ripple_mV_{Vin}V'] = vr * 1e3
psrr_db = 20                                  # conservative TLV1117LV PSRR at 0.7 MHz (est.)
R['v33_ripple_mV'] = R['buck_ripple_mV_12V'] * 10 ** (-psrr_db / 20)
# load budget (typ / design max, mA)
loads33 = {'STM32F407 @168 MHz + ETH MAC': (90, 120), 'LAN8742A 100BASE-TX (incl. TX centre tap)': (45, 60),
           'TCXO': (3, 5.5), 'SN74LVC2G34': (1, 2), 'ISO7740 side 1 (x4)': (13.2, 18.8), 'LEDs (x3, 1 k)': (4, 5)}
loadsISO = {'ISO7740 side 2 (x4)': (9.2, 14.4), '74AC541 + TSSOP-20': (1, 2), 'Camera trigger inputs 8 x 10 mA (peak)': (2, 80),
            'Strobe pull-ups 8 x 4.7 k': (4, 8.5), 'RFM minimum-load resistor 270 R': (18.5, 18.5)}
i33 = [sum(v[k] for v in loads33.values()) for k in (0, 1)]
iiso = [sum(v[k] for v in loadsISO.values()) for k in (0, 1)]
eta_rfm, eta_buck = 0.75, 0.85
i5 = [i33[k] + iiso[k] / eta_rfm + 1.5 for k in (0, 1)]   # + green LED (2.2 k)
R.update(loads33=loads33, loadsISO=loadsISO, i33=i33, iiso=iiso, i5=i5)
R['p_ldo_W'] = [(Vo - 3.3) * i / 1e3 for i in i33]
R['ldo_dT_C'] = [p * 60 for p in R['p_ldo_W']]           # SOT-223 on copper ~60 C/W
for Vin in (7, 12):
    R[f'iin_mA_{Vin}V'] = [Vo * i / eta_buck / Vin for i in i5]
R['rfm_peak_W'] = 5 * iiso[1] / 1e3
# ---------- 5. IPC-2221 trace widths (external, 1 oz = 35 um, dT = 10 C) ----------
def width_mm(I, dT=10, k=0.048, oz=1):
    A = (I / (k * dT ** 0.44)) ** (1 / 0.725)          # mil^2
    return A / (1.378 * oz) * 0.0254
tw = {}
for name, I in [('VIN path (design = PTC trip ~1.0 A)', 1.0), ('5 V main', 0.6), ('3.3 V branches', 0.3), ('5V_ISO', 0.2)]:
    tw[name] = (I, width_mm(I))
R['trace_width'] = tw
# ---------- 6. clock / PTP ----------
R['ptp_addend'] = int(2 ** 32 * (1e9 / 20) / 168e6)
R['ptp_resolution_ns'] = 20
json.dump(R, open('calcs.json', 'w'), indent=1, default=float)
for k, v in R.items(): print(k, '=', v)
