#!/usr/bin/env python3
"""Run deterministic C2 mock tests and freestanding RISC-V compilation.

No hardware is accessed. Artifacts are created in a temporary directory.
Requires cc, c++, and clang (with RISC-V backend); no board SDK is required.
"""
import argparse
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]

# Independent byte-address map from the C2 board.h / works register tables.
BLOCKS = {
    "GPIO": (8, "GPIO0", 0x10000000, "DR DDR"),
    "SYS_UART": (8, "UART0", 0x10001000, "CLKDIV DATA"),
    "HP_UART": (20, "UART1", 0x20003000, "LCR DIV TRX FCR LSR"),
    "TIM": (12, "TIM0", 0x10002000, "CONFIG VALUE DATA"),
    "PSRAM": (8, "PSRAM0", 0x10004000, "WC CHD"),
    "ARCHINFO": (12, "ARCHINFO0", 0x20001000, "SYS IDL IDH"),
    "RNG": (12, "RNG0", 0x20002000, "CTRL SEED VAL"),
    "PWM": (36, "PWM0", 0x20004000, "CTRL PSCR RESERVED_CNT CMP CR0 CR1 CR2 CR3 STAT"),
    "PS2": (12, "PS20", 0x20005000, "CTRL DATA STAT"),
    "I2C": (24, "I2C0", 0x20006000, "CTRL PSCR TXR RXR CMD SR"),
    "QSPI": (44, "QSPI0", 0x20007000, "STATUS CLKDIV CMD ADR LEN DUM TXFIFO RESERVED0 RXFIFO INTCFG INTSTA"),
}

# Frozen least-significant-first field widths from the documented layouts.
# UNKNOWN retains uncertainty; it must not be silently reinterpreted as an
# undocumented feature. This oracle does not depend on header formatting.
FIELDS = {
    'ARCHINFO': {'SYS': 'SRAM:8 CLOCK:12 RESERVED0:12', 'IDL': 'CUST:6 PROCESS:16 VENDOR:8 TYPE:2', 'IDH': 'DATE:24 RESERVED0:8'},
    'GPIO': {r: ' '.join(f'PIN{i}:1' for i in range(16)) + ' UNKNOWN0:16' for r in ('DR', 'DDR')},
    'SYS_UART': {'CLKDIV': 'CLKDIV:32', 'DATA': 'DATA:8 UNKNOWN0:24'},
    'HP_UART': {'LCR': 'RXIE:1 TXIE:1 PEIE:1 WLS:2 STB:1 PEN:1 PS:2 RESERVED0:23', 'DIV': 'DIV:16 RESERVED0:16', 'TRX': 'DATA:8 RESERVED0:24', 'FCR': 'RF_CLR:1 TF_CLR:1 RX_TRG_LEVL:2 RESERVED0:28', 'LSR': 'RXIP:1 TXIP:1 PEIP:1 DR:1 PE:1 THRE:1 TEMT:1 EMPT:1 FULL:1 RESERVED0:23'},
    'TIM': {'CONFIG': 'ENABLE:1 UNKNOWN0:7 LOAD:1 UNKNOWN1:23', 'VALUE': 'VALUE:32', 'DATA': 'DATA:32'},
    'PSRAM': {'WC': 'WC:32', 'CHD': 'CHD:32'},
    'RNG': {'CTRL': 'EN:1 RESERVED0:31', 'SEED': 'SEED:32', 'VAL': 'VAL:32'},
    'PWM': {'CTRL': 'OVIE:1 EN:1 CLR:1 RESERVED0:29', **{r: f'{r}:16 RESERVED0:16' for r in ('PSCR', 'CNT', 'CMP', 'CR0', 'CR1', 'CR2', 'CR3')}, 'STAT': 'OVIF:1 RESERVED0:31'},
    'PS2': {'CTRL': 'ITN:1 EN:1 RESERVED0:30', 'DATA': 'DATA:8 RESERVED0:24', 'STAT': 'ITF:1 RESERVED0:31'},
    'I2C': {'CTRL': 'RESERVED0:6 IEN:1 EN:1 RESERVED1:24', 'PSCR': 'PSCR:16 RESERVED0:16', 'TXR': 'DATA:8 RESERVED0:24', 'RXR': 'DATA:8 RESERVED0:24', 'CMD': 'IACK:1 RESERVED0:2 ACK:1 WR:1 RD:1 STO:1 STA:1 RESERVED1:24', 'SR': 'IF:1 TIP:1 RESERVED0:3 AL:1 BSY:1 RXK:1 RESERVED1:24'},
    'QSPI': {'STATUS': 'BUSY:1 GO:1 UNKNOWN0:2 RESET:1 UNKNOWN1:3 LOCK:1 UNKNOWN2:23', 'CLKDIV': 'CLKDIV:32', 'CMD': 'UNKNOWN0:16 CS:3 UNKNOWN1:13', 'ADR': 'ADR:32', 'LEN': 'UNKNOWN0:32', 'DUM': 'DUM:32', 'TXFIFO': 'DATA:32', 'RXFIFO': 'DATA:32', 'INTCFG': 'UNKNOWN0:32', 'INTSTA': 'UNKNOWN0:32'},
}


def execute(argv, source=None):
    subprocess.run([str(arg) for arg in argv], input=source, text=True, check=True)


def layout_source():
    lines = ['#include "c2.h"', '#include <stddef.h>',
             '#ifdef __cplusplus', '#define CHECK static_assert', '#else',
             '#define CHECK _Static_assert', '#endif']
    for kind, (size, instance, base, members) in BLOCKS.items():
        typ = f"C2_{kind}_TypeDef"
        lines.append(f'CHECK(sizeof({typ}) == {size}, "block size");')
        lines.append(f'CHECK(C2_{instance}_BASE == 0x{base:x}u, "base");')
        for index, member in enumerate(members.split()):
            lines.append(f'CHECK(offsetof({typ}, {member}) == {index * 4}, "offset");')
    lines.append('CHECK(C2_TIM1_BASE == 0x10003000u, "timer 1 base");')
    expected = {}
    declared = {}
    for header in sorted((ROOT / 'inc').glob('*.h')):
        for body, typ in re.findall(r'typedef union\s*\{(.*?)\}\s*(C2_\w+_TypeDef);',
                                    header.read_text(), re.S):
            declared[typ] = re.findall(r'uint32_t\s+(\w+)\s*:\s*(\d+)\s*;', body)
    for peripheral, registers in FIELDS.items():
        for register, spec in registers.items():
            typ = f'C2_{peripheral}_{register}_TypeDef'
            fields = [tuple(field.split(':')) for field in spec.split()]
            assert declared.get(typ) == fields, (typ, declared.get(typ), fields)
            lines.append(f'CHECK(sizeof({typ}) == 4, "register size");')
            low = 0
            for field, width in fields:
                width = int(width)
                value = (1 << width) - 1
                index = len(expected)
                expected[index] = value << low
                lines.append(f'uint32_t layout_{index}(void) {{ {typ} r = {{0}}; '
                             f'uint32_t v; r.BITS.{field} = 0x{value:x}u; '
                             '__builtin_memcpy(&v, &r.BITS, 4); return v; }')
                low += width
            assert low == 32, typ
    assert len(expected) == 154, 'Register field coverage changed; review the layout test'
    main = 'int main(void) {\n' + ''.join(
        f'if (layout_{index}() != 0x{value:x}u) return 1;\n'
        for index, value in expected.items()) + 'return 0; }\n'
    return '\n'.join(lines) + '\n', main, expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='cc')
    parser.add_argument('--cxx', default='c++')
    parser.add_argument('--clang', default='clang')
    parser.add_argument('--sanitize', action='store_true', help='enable host ASan/UBSan')
    args = parser.parse_args()
    include = ['-I', ROOT / 'inc']
    warnings = ['-Wall', '-Wextra', '-Werror', '-pedantic']
    sources = sorted((ROOT / 'src').glob('*.c'))
    suites = ('misc', 'gpio_uart', 'i2c', 'timer_pwm', 'qspi_psram')
    with tempfile.TemporaryDirectory(prefix='c2-tests-') as temporary:
        temp = pathlib.Path(temporary)
        for suite in suites:
            test = ROOT / 'tests' / f'test_{suite}.c'
            mock = ROOT / 'tests' / f'{suite}_mock.h'
            exe = temp / suite
            command = [args.cc, '-std=c11', '-O2', *warnings, *include,
                       '-include', mock, *sources, test, '-o', exe]
            if args.sanitize:
                command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
            execute(command)
            execute([exe])
        layout, entry, expected = layout_source()
        for compiler, language, standard in ((args.cc, 'c', 'c11'), (args.cxx, 'c++', 'c++11')):
            flags = [compiler, '-x', language, '-std=' + standard, *warnings, *include]
            for header in sorted((ROOT / 'inc').glob('*.h')):
                execute([*flags, '-fsyntax-only', '-'],
                        f'#include "{header.name}"\n#include "{header.name}"\n')
            exe = temp / 'layout'
            execute([*flags, '-o', exe, '-'], layout + entry)
            execute([exe])
            print(f'{standard}: standalone headers, register offsets/sizes, 154 field masks passed', flush=True)
        # Resolve every public driver prototype from C++ against the actual C
        # objects. Merely parsing extern declarations cannot catch missing
        # implementations or accidental C++ name mangling.
        objects = []
        for source in sources:
            obj = temp / (source.stem + '-host.o')
            execute([args.cc, '-std=c11', '-O2', *warnings, *include, '-c', source, '-o', obj])
            objects.append(obj)
        functions = sorted(set(re.findall(
            r'c2_status_t\s+(c2_\w+)\s*\(',
            '\n'.join(header.read_text() for header in (ROOT / 'inc').glob('*.h')))))
        link_source = temp / 'link.cpp'
        link_source.write_text('#include "c2.h"\n' + ''.join(
            f'auto link_check_{i} = &{function};\n' for i, function in enumerate(functions)) +
            'int main() { return 0; }\n')
        execute([args.cxx, '-std=c++11', *warnings, *include, link_source,
                 *objects, '-o', temp / 'link'])
        execute([temp / 'link'])
        print(f'C++ link: {len(functions)} public driver functions resolved', flush=True)
        for assignment in ('C2_I2C0->SR.WORD = 0', 'C2_I2C0->SR.BITS.BSY = 0',
                           'C2_PS20->DATA.WORD = 0', 'C2_UART1->LSR.BITS.DR = 0'):
            result = subprocess.run([str(arg) for arg in
                                     [args.cc, '-std=c11', *include, '-x', 'c', '-fsyntax-only', '-']],
                                    input='#include "c2.h"\nvoid f(void) { ' + assignment + '; }\n',
                                    text=True, capture_output=True)
            assert result.returncode != 0, 'Read-only register accepted a write: ' + assignment
        print('Read-only WORD/BITS write rejection checks passed', flush=True)
        target = [args.clang, '--target=riscv32-unknown-elf', '-march=rv32im', '-mabi=ilp32',
                  '-std=c11', '-ffreestanding', '-fno-builtin', '-Os', *warnings, *include]
        for source in sources:
            execute([*target, '-c', source, '-o', temp / (source.stem + '.o')])
        result = subprocess.run([str(arg) for arg in [*target, '-S', '-emit-llvm', '-x', 'c', '-o', '-', '-']],
                                input=layout, text=True, capture_output=True, check=True)
        actual = {int(i): int(v) & 0xffffffff for i, v in re.findall(
            r'define[^\n]*@layout_(\d+)\(.*?ret i32 (-?\d+)', result.stdout, re.S)}
        assert actual == expected, 'RISC-V bit-field allocation differs from declared ranges'
        print(f'RV32IM/ILP32: {len(sources)} drivers compiled; 154 field masks passed', flush=True)
    print('All C2 software tests passed (not a hardware qualification).')


if __name__ == '__main__':
    main()
