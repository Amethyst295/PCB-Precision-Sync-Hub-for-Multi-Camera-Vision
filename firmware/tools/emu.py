import sys, collections
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile
from capstone import Cs, CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_MCLASS
elf=ELFFile(open(sys.argv[1],'rb'))
sym={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
mu=Uc(UC_ARCH_ARM, UC_MODE_THUMB|UC_MODE_MCLASS); mu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4) if False else None
mu.mem_map(0x08000000,0x100000); mu.mem_map(0x20000000,0x20000)
for seg in elf.iter_segments():
    if seg['p_type']=='PT_LOAD' and seg['p_filesz']:
        mu.mem_write(seg['p_paddr'], seg.data())
# copy .data from flash load to RAM and zero bss
d=elf.get_section_by_name('.data')
if d and d['sh_size']: mu.mem_write(d['sh_addr'], d.data())
# enable FPU
try:
    mu.reg_write(UC_ARM_REG_C1_C0_2, mu.reg_read(UC_ARM_REG_C1_C0_2)|(0xf<<20))
except Exception as e: pass
try: mu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
except Exception as e: print('fpexc',e)
cs=Cs(CS_ARCH_ARM, CS_MODE_THUMB|CS_MODE_MCLASS)
cnt=collections.Counter(); ninst=[0]; heavy=[0]
def hook(uc,addr,size,ud):
    ninst[0]+=1
    b=uc.mem_read(addr,size)
    for i in cs.disasm(bytes(b),addr):
        m=i.mnemonic
        if m.startswith('vdiv') or m.startswith('vsqrt'): heavy[0]+=13
        elif m.startswith(('ldm','stm','push','pop','vpush','vpop','vldm','vstm')): heavy[0]+= max(0,len(i.op_str.split(','))-1)
        elif m.startswith(('b','cb')) : heavy[0]+=1  # approx pipeline refill when taken
mu.hook_add(UC_HOOK_CODE,hook)
RET=0x08000000|0xF0001
mu.mem_map(0x0F000000,0x1000) if False else None
def call(fn,*args,fargs=None):
    ninst[0]=0; heavy[0]=0
    mu.reg_write(UC_ARM_REG_SP,0x2001FF00)
    regs=[UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]
    for r,a in zip(regs,args): mu.reg_write(r,a)
    if fargs is not None: mu.reg_write(UC_ARM_REG_S0, fargs)
    stop=0x080FFF00
    mu.mem_write(stop, b'\x00\xbf\x00\xbf')
    mu.reg_write(UC_ARM_REG_LR, stop|1)
    mu.emu_start(sym[fn]|1, stop, count=50_000_000)
    return ninst[0], heavy[0]
call('bench_init')
res=[]
for k in range(80):
    res.append(call('bench_servo',k,0))
import struct
print('servo first',res[0],'steady (k>=64) mean',sum(r[0] for r in res[64:])/16,'max',max(r[0] for r in res), 'heavyextra mean',sum(r[1] for r in res[64:])/16)
t=[call('bench_tree',0,fargs=struct.unpack('<I',struct.pack('<f',float(k)))[0]) for k in range(10)]
print('tree',t)
# attribution
funcs=sorted([(s['st_value']&~1,s['st_size'],s.name) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_FUNC' and s['st_size']>0])
import bisect
starts=[f[0] for f in funcs]
attr=collections.Counter()
def hook2(uc,addr,size,ud):
    i=bisect.bisect_right(starts,addr)-1
    attr[funcs[i][2] if i>=0 and addr<funcs[i][0]+funcs[i][1] else '?']+=1
h=mu.hook_add(UC_HOOK_CODE,hook2)
for k in range(80,96): call('bench_servo',k,0)
tot=sum(attr.values())
for n,c in attr.most_common(15): print(f"{n:22s} {c/16:9.0f} {100*c/tot:5.1f}%")
