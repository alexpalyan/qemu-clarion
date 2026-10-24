# coredll export lists

Named exports of `coredll.dll` on Clarion head units, one name per line,
sorted. Use them to check that a program built for these units imports only
functions the unit's coredll provides: a missing import makes Windows CE
refuse to load the program, silently.

| File | CPU, OS | Kernel | Names | Checked on |
|---|---|---|---|---|
| `qy7-main.txt` | SH-4, Windows CE 5.0 | main OS kernel in NOR | 1765 | QY7201NE (X110ECNI.158), QY7214NL (X112JANI.141), QY7221NL (X112ECNI.100) |
| `qy7-updater.txt` | SH-4, Windows CE 5.0 | updater kernel on the map card | 1651 | QY7201NE, QY7214NL (QSD-398-100), QY7221NL (QSD-357-101) |
| `qy8-main.txt` | ARM, Windows CE 6/7 | main OS kernel in NOR | 2207 | QY8202NA, QY8202NC, QY8202PB, QY8252NF (G214), QY8600NE, QY8602NB, QY8652NB (G218) |

Within each row the lists of all checked units are identical, although the
coredll binaries differ between firmware builds. `qy7-updater.txt` is a strict
subset of `qy7-main.txt`, so a program that imports only names from it runs
in both kernels. The updater kernel's list for QY8 is not here yet.

Compared with the Windows CE 5.0 Standard SDK `coredll.lib` for SH4 (1708
names), the QY7 updater coredll lacks 96 SDK names (mostly `Crypt*` and
`acm*`) and adds 39. Neither has `sqrtf`; of the `float` math variants only
`ceilf`, `floorf` and `fmodf` exist.

How the lists were made: the QY7 coredll was extracted from the ROM image
(XIP region at NOR `0x40000`, updater banks at card `0x3E00000`) and its PE
export name table listed with `objdump -p`; the QY8 one was read from the
export table of the uncompressed kernel XIP region at NOR `0x1C0000`.
