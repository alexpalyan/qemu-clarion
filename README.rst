============================
Clarion head units in QEMU
============================

This is a fork of `QEMU <https://www.qemu.org/>`_ that emulates the boards
of Clarion head units fitted to the Nissan Leaf, well enough to boot their
original Windows CE firmware. It exists to study and repair these units
without risking the one in the car.

No firmware is included. You supply dumps taken from your own unit.

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
   * - QY7221NL
     - SH-4A
     - ``clarion-qy7``
     - ``qemu-system-sh4``
     - first-stage Windows CE 5.0 kernel and its drivers start; serial
       console only, no display or SD card yet

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

Running a unit
==============

``qemu-clarion`` is a small launcher. It reads the unit model from the
``PROD`` block of the NOR image, picks the emulator and machine for it, and
starts QEMU. You never pass ``-M`` or ``-drive`` yourself.

.. code-block:: shell

  qemu-clarion [-n] [--read-only] [--headless] NOR [SD] [QEMU arguments...]

.. code-block:: shell

  build-release/qemu-clarion flash.bin                          # NOR only
  build-release/qemu-clarion flash.bin map-card.img             # NOR and SD card
  build-release/qemu-clarion --read-only flash.bin map-card.img # leave both files untouched
  build-release/qemu-clarion --headless flash.bin               # no window
  build-release/qemu-clarion -n flash.bin map-card.img          # print the QEMU command and exit

.. list-table::
   :header-rows: 1

   * - Argument
     - Meaning
   * - ``NOR``
     - raw dump of the unit's NOR flash: 64 MiB for QY8652NB and QY8202NA,
       8 MiB for QY7221NL
   * - ``SD``
     - optional raw image of the SD card for the front slot; rejected for
       units whose machine has no SD controller yet (QY7221NL)
   * - ``--read-only``
     - open both images with ``snapshot=on``: guest writes go to temporary
       storage and the files do not change
   * - ``--headless``
     - run without a display window (``-nographic``)
   * - ``-n``
     - print the command line that would be run
   * - anything after the images
     - passed to QEMU unchanged, for example ``-machine dipsw=1``

**By default the guest writes to the images you pass**, as the real unit
writes to its flash and card, and those changes survive a restart. Work on
copies, or use ``--read-only``. Without ``--read-only`` the launcher refuses
an image it cannot write to.

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

Machine properties go in a ``-machine name=value`` argument and merge with
the machine the launcher selected. ``dipsw`` selects the boot mode on every
machine (``dipsw=1`` adds a step-by-step boot log on QY8). The
``clarion-qy8`` properties, serial port layout, pointer input, diagnostics
and the MIRROR renderer are described in
`docs/clarion/clarion-qy8.md <docs/clarion/clarion-qy8.md>`_.

Limitations
===========

* The PowerVR GPU is not modelled. GPU-rendered content reaches the window
  through a built-in software path (``render=cpu``, the default).
* Not modelled on QY8: the vehicle CAN bus beyond the controller itself,
  USB, I2C devices other than the touch controller.
* QY7221NL has no display, SD card or watchdog reset yet, so it stops where
  its launcher would ask for the map card.

Relation to QEMU
================

The fork lives on the ``clarion_qy8`` branch and follows upstream QEMU; the
Clarion code is in ``hw/arm/clarion_qy8.c``, ``hw/sh4/clarion_qy7.c``, the
``clarion_*`` devices under ``hw/``, and ``contrib/clarion/``. Everything
else is unmodified QEMU: see https://www.qemu.org/ for its documentation
and https://gitlab.com/qemu-project/qemu for the source. QEMU is licensed
under the GNU General Public License, version 2 (see ``COPYING`` and
``LICENSE``).

This project is not affiliated with Nissan or Clarion.
