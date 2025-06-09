# Soundial v1 – Assembly Instructions

These instructions will walk you through assembling your own Soundial v1 metronome.

---

## 🖨️ Step 1: 3D Printing the Parts

Print the following STL files using your preferred 3D printer (tested on FlashForge Adventurer 4):

- `Soundial-v1-Body.stl`
- `Soundial-v1-Button.stl`
- `Soundial-v1-Diffuser.stl`
- `Soundial-v1-Face-Cover.stl`

All files are optimized for standard FDM printing and should work on most hobbyist printers.

---

## 🔩 Step 2: Install Threaded Inserts

Use a soldering iron or insert-setting tool to heat-set the threaded inserts into the appropriate holes in the 3D printed parts:

- M3 and M2.5 inserts go into their designated slots for mounting internal components.

---

## 🔧 Step 3: Wiring and Soldering

1. **Crimp Wires:** Cut and crimp wire leads for connections using ring terminals.
2. **OLED Display:** Solder the wire leads to the pins of the OLED display (I2C interface).
3. **Power Switch:**
   - Cut the positive (red) wire between the battery and its JST connector.
   - Solder the SPDT switch inline to allow power cutoff to the Circuit Playground.

---

## ⚡ Step 4: Final Assembly

1. Mount the **Circuit Playground** inside the main body using screws.
2. Connect the **OLED display** and secure it behind the face cover.
3. Plug in the **battery** via the JST connector.
4. Fit all parts together and secure with screws through the pre-threaded inserts.
5. Snap on the diffuser and press-fit the button into place.

---

## ✅ Final Check

- Flash the firmware from this repo to the Circuit Playground.
- Toggle the switch and confirm LED ring + click sound.
- Secure with clamp or place on table — you're ready to drum.

