============================
Clarion head units in QEMU
============================

This is a fork of `QEMU <https://www.qemu.org/>`_ that emulates the boards
of Clarion head units fitted to the Nissan Leaf, well enough to boot their
original Windows CE firmware. It exists to study and repair these units
without risking the one in the car.

No firmware is included. You supply dumps taken from your own unit, usually
with `LeafSDTools <https://github.com/developerfromjokela/leafsdtools>`_,
which runs from an SD card in the unit's update mode and writes the NOR
flash to the card.

.. list-table::
   :header-rows: 1

   * - Unit
     - CPU
     - Machine
     - Emulator
     - State
   * - QY8652NB
     - ARM (Renesas R-Car)
     - ``clarion-qy8``
     - ``qemu-system-arm``
     - boots to the user interface: consent screen, menu, touch input,
       map screen with the map card
   * - QY8202NA
     - ARM (Renesas R-Car)
     - ``clarion-qy8``
     - ``qemu-system-arm``
     - boots to the consent screen
   * - QY8602NB, QY8600NE
     - ARM (Renesas R-Car)
     - ``clarion-qy8``
     - ``qemu-system-arm``
     - 2018+ Leaf (ZE1), European and US units, same board as QY8652NB
       (TMA460 touch); boot to the consent screen
   * - QY8202PB
     - ARM (Renesas R-Car)
     - ``clarion-qy8``
     - ``qemu-system-arm``
     - e-NV200 (ME0), same board as QY8202NA (TMA616 touch); boots to the
       consent screen
   * - QY8202NC, QY8252NF
     - ARM (Renesas R-Car)
     - ``clarion-qy8``
     - ``qemu-system-arm``
     - not in the board table, run with the QY8 defaults (TMA460 touch);
       boot to the consent screen, touch not checked
   * - QY7221NL
     - SH-4A
     - ``clarion-qy7``
     - ``qemu-system-sh4``
     - boots to the user interface with the paired map card: consent
       screen, touch input, map screen; MENU, MAP/VOICE, STATUS and ZERO
       EMISSION keys arrive over the vehicle CAN bus

Other units run on the machine of their series, chosen by the ``QY7`` or
``QY8`` prefix of the model, with a warning that the unit has not been
tested.

Developed and tested on macOS (Apple Silicon). A Linux build should work
the same way but is not covered here.

Building
========

Dependencies on macOS: Xcode Command Line Tools, Python 3, and
``brew install ninja pkg-config glib pixman``.

.. code-block:: shell

  mkdir -p build-release && cd build-release
  ../configure --target-list=arm-softmmu,sh4-softmmu \
      --disable-werror --enable-plugins --disable-docs
  ninja qemu-system-arm qemu-system-sh4 qemu-clarion
  cd ..

Use this optimized build to run units. A debug build
(``../configure … --enable-debug`` in a separate ``build`` directory) is
only for QEMU assertions and symbols: the guest is noticeably slower in it
and the QY8 user interface misses its first screen.

Releases / binary packages
==========================

Every tagged release (``v*``) on the GitHub Releases page carries ready-made
archives, ``qemu-clarion-<tag>-<os>-<arch>.tar.gz``, for macOS (arm64) and
Linux (x86_64). Each one holds the launcher, both emulation engines
(``libexec/``) and all the shared libraries they need (``lib/``), so nothing
else has to be installed.

.. code-block:: shell

  tar xzf qemu-clarion-<tag>-darwin-arm64.tar.gz
  cd qemu-clarion-<tag>-darwin-arm64
  xattr -dr com.apple.quarantine .      # macOS only: the binaries are not notarized
  ./qemu-clarion NOR [SD]

The binaries are ad-hoc signed on macOS. The archives are built by
``.github/workflows/release.yml`` using ``scripts/clarion/package.sh``.

Running a unit
==============

``qemu-clarion`` is a small launcher. It reads the unit model from the
``PROD`` block of the NOR image, picks the emulator and machine for it, and
starts QEMU. You never pass ``-M`` or ``-drive`` yourself.

.. code-block:: shell

  qemu-clarion [-n] [--writable] [--headless] [--log FILE] [--dbgprint FILE] NOR [SD] [QEMU arguments...]
  qemu-clarion --help

.. code-block:: shell

  build-release/qemu-clarion flash.bin                          # NOR only
  build-release/qemu-clarion flash.bin map-card.img             # NOR and SD card
  build-release/qemu-clarion --writable flash.bin map-card.img  # write both files in place
  build-release/qemu-clarion --headless flash.bin               # no window
  build-release/qemu-clarion --log run.log flash.bin             # launcher and engine output to one file
  build-release/qemu-clarion --dbgprint guest.log flash.bin     # capture guest debug output
  build-release/qemu-clarion -n flash.bin map-card.img          # print the QEMU command and exit

.. list-table::
   :header-rows: 1

   * - Argument
     - Meaning
   * - ``NOR``
     - raw dump of the unit's NOR flash: 64 MiB for the QY8 units,
       8 MiB for QY7221NL
   * - ``SD``
     - optional raw image of the SD card for the front slot, including
       QY7221NL (SDHI0)
   * - ``--writable``
     - write to NOR and SD in place (default: ``snapshot=on``, an
       in-memory overlay discarded on exit)
   * - ``--headless``
     - run without a display window (``-nographic``)
   * - ``--log FILE``
     - send the launcher's and the engine's stdout/stderr to FILE
   * - ``--dbgprint FILE``
     - record Windows CE debug output in FILE using the bundled plugin
   * - ``-n``
     - print the command line that would be run
   * - anything after the images
     - passed to QEMU unchanged, for example ``-machine dipsw=1``; a
       passed-through ``-M`` replaces the one the launcher picked, so
       board properties (for example ``gps=off``) can be set too

**By default both images are opened read-only** (``snapshot=on``): guest
writes go to an in-memory overlay and the files do not change. Pass
``--writable`` to write to them in place, as the real unit writes to its
flash and card. Without ``--writable`` the launcher does not touch either
file's permissions; with it, it refuses an image it cannot write to.

What the launcher sets up unless you override it:

* a display window and the guest console on the terminal
  (``-serial mon:stdio``; ``Ctrl-A C`` switches to the QEMU monitor,
  ``Ctrl-A X`` quits). Passing your own ``-serial``, ``-display`` or
  ``-nographic`` turns this off;
* on macOS, a visible mouse pointer over the window
  (``-display cocoa,show-cursor=on``); a click is a touch;
* for QY7221NL, ``-icount shift=2``. That kernel reprograms its tick timer
  in a way that does not survive host-clock timing; pass your own
  ``-icount`` to change it.

The QY8 screen is not immediate: the consent screen appears after roughly
45 to 100 seconds, depending on the host.

Machine properties go in a ``-machine name=value`` argument, which replaces
the one the launcher would otherwise have picked. ``dipsw`` selects the boot mode on every
machine (``dipsw=1`` adds a step-by-step boot log on QY8). The
``clarion-qy8`` properties, serial port layout, pointer input, diagnostics
and the MIRROR renderer are described in
`docs/clarion/clarion-qy8.md <docs/clarion/clarion-qy8.md>`_; the
``clarion-qy7`` properties and its front-panel keys in
`docs/clarion/clarion-qy7.md <docs/clarion/clarion-qy7.md>`_.

Limitations
===========

* The PowerVR GPU is not modelled. GPU-rendered content reaches the window
  through a built-in software path (``render=cpu``, the default).
* Not modelled on QY8: the vehicle CAN bus beyond the controller itself,
  USB, I2C devices other than the touch controller.
* QY7221NL locks the SD card with the password from the ``PROD`` block at
  power-up (``paired-card=on``), as a card paired with the unit would be.
  Audio is not modelled: the micom reports audio off, so the audio source
  and volume keys have no visible effect and are not mapped yet. Watchdog
  reset is not modelled.

Relation to QEMU
================

The fork lives on the ``master`` branch and follows upstream QEMU; the
Clarion code is in ``hw/arm/clarion_qy8.c``, ``hw/sh4/clarion_qy7.c``, the
``clarion_*`` devices under ``hw/``, and ``contrib/clarion/``. Everything
else is unmodified QEMU: see https://www.qemu.org/ for its documentation
and https://gitlab.com/qemu-project/qemu for the source. QEMU is licensed
under the GNU General Public License, version 2 (see ``COPYING`` and
``LICENSE``).

This project is not affiliated with Nissan or Clarion.
