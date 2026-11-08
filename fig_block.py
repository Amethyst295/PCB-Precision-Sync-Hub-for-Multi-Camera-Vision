import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
fig, ax = plt.subplots(figsize=(11, 6.4), dpi=200)
ax.set_xlim(0, 110); ax.set_ylim(0, 64); ax.axis('off')
INK = '#1f2933'; MUTED = '#52606d'; ACC = '#2563eb'; ISO = '#e8f1ff'; BOX = '#f7f8fa'; WARN = '#b45309'
def box(x, y, w, h, title, lines, fc=BOX, ec='#9aa5b1', bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.4,rounding_size=1.2', fc=fc, ec=ec, lw=1.4 if bold else 1))
    ax.text(x + 1.2, y + h - 2.2, title, fontsize=9.5, fontweight='bold', color=INK, va='top')
    for i, l in enumerate(lines):
        ax.text(x + 1.2, y + h - 5.4 - i * 2.6, l, fontsize=7.6, color=MUTED, va='top')
def arrow(x1, y1, x2, y2, label='', both=False, color=INK, lx=0, ly=1.0):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle='<|-|>' if both else '-|>', mutation_scale=9, color=color, lw=1))
    if label: ax.text((x1 + x2) / 2 + lx, (y1 + y2) / 2 + ly, label, fontsize=7, color=MUTED, ha='center')
# isolation barrier
ax.add_patch(FancyBboxPatch((80, 12.5), 29, 50, boxstyle='round,pad=0.2,rounding_size=1.5', fc=ISO, ec='none'))
ax.plot([80, 80], [12, 63], ls=(0, (4, 3)), color=ACC, lw=1.3)
ax.text(95, 61.2, 'Isolated (5V_ISO)', fontsize=8, color=ACC, ha='center', fontweight='bold')
ax.text(40, 61.2, 'Board side (3V3 / GND)', fontsize=8, color=MUTED, ha='center', fontweight='bold')
box(1, 44, 21, 14, 'Ethernet', ['HR911105A RJ45 + magnetics', 'LAN8742A PHY (RMII)', '49.9 R terminations'])
box(1, 25, 21, 14, 'Clock', ['ECS-TXO-2016 25 MHz TCXO', '±2.5 ppm (-40…85 °C)', 'SN74LVC2G34 fan-out buffer'])
box(30, 25, 25, 33, 'STM32F407VET6', ['Cortex-M4F, 168 MHz', 'ETH MAC + IEEE 1588 clock', 'PTP target time → TIM2 (ITR1)', 'TIM3/TIM4: 8 trigger pulses', 'TIM1/TIM9/TIM12: 8 strobe', '  captures (exposure latency)', 'MLP bias corrector (PTP servo)', 'Decision-tree fault monitor', 'USART1 telemetry · SWD'], fc='#eef4ff', ec=ACC, bold=True)
box(60, 40, 22, 18, 'ISO7740F ×2', ['trigger ch 1–8', 'tsk(pp) ≤ 4.5 ns', '5 kVrms reinforced'])
box(60, 16, 22, 18, 'ISO7740F ×2', ['strobe ch 1–8', '(camera → MCU)'])
box(85, 40, 23, 18, '74AC541 driver', ['±24 mA per output', '82 R series', '10 k pull-down', 'SRV05-4 ESD ×2'])
box(85, 16, 23, 18, 'Camera ports ×8', ['JST B4B-XH-A', 'TRIG·STB·5V·GND', '100 R + 4.7 k (STB)', 'SRV05-4 ESD ×2'])
box(1, 1.5, 82, 10, 'Power', ['7–12 V terminal block → PTC 0.5 A → reverse-polarity Schottky + SMBJ33A → LMR16006X buck (5.05 V, 0.7 MHz)', '→ TLV1117LV33 LDO (3.3 V) → ferrite split: 3V3 (digital) · 3V3A (PHY/VDDA) · 3V3_CLK (TCXO)'])
box(85, 1.5, 23, 10, 'RFM-0505S', ['5 V → 5V_ISO, 1 W, 1 kVDC', '270 R minimum load'], fc=ISO, ec=ACC)
arrow(22.5, 51, 30, 51, 'RMII', both=True)
arrow(22.5, 32, 30, 32, '25 MHz')
arrow(11, 39.4, 11, 44, '25 MHz', lx=4, ly=-0.6)
arrow(55.4, 49, 60, 49, '8×')
arrow(60, 29, 55.4, 29, '8×')
arrow(82.4, 49, 85, 49)
arrow(96, 39.6, 96, 34.4, '', color=INK)
arrow(85, 25, 82.4, 25)
arrow(83.4, 6.5, 85, 6.5)
#ax.text(55, 0.0, 'Figure 1 · Hardware block diagram as captured in the final Proteus schematic', fontsize=7.5, color=MUTED, ha='center')
plt.savefig('fig1_block.png', bbox_inches='tight', facecolor='white')
