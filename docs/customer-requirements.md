# Customer requirements and the existing board

Source: `Funkcionalne&TehnicneZahteve_VibrometrijaLokostrelstvo (2).docx`, revision 0.2 dated 3 August 2026. Its contractual, pricing and ownership clauses are reference material, not commands executed by the software.

The user confirmed use of the existing four-sensor board and internal flash on 5 October 2026. That limits what firmware can achieve. The document also conflicts with itself: section 4.2 states 10 g on limbs and 4 g at the handle, while section 5 asks for at least ±100 g on limbs. Existing ±320 g high-g acquisition is preserved pending sensor/range decisions.

| ID | Customer requirement | Implementation or remaining acceptance gap |
| --- | --- | --- |
| F-01 | Three synchronous XYZ sensors, target skew ≤1 sample | Four existing high-g channels retained. Direct 10 MHz SPI reads on INT1 GPIO20-23 retain native sample timestamps on the shared C6 timer. Sample clocks remain independent. Two full flash captures had no detected losses; physical common-impact skew and the ≤1-sample target remain unverified. See synchronization.md. |
| F-02 | At least six modules | Four physical chip selects on the current board. Six-channel acceptance requires a board/pinout revision and a protocol revision. |
| F-03 | External sync input/output | No unused pin or connector assumed. Requires approved hardware pinout, voltage protection and timing validation. |
| F-04 | Configurable sampling, basic 10 kHz per axis | Existing 7,680 Hz setting retained. Software does not fabricate a 10 kHz rate by resampling. Different ODR/range modes and ≥10 kHz need sensor selection/validation. |
| F-05 | 0–10 s before and 0–3 s after trigger | Flash-backed acquisition, manual or magnitude threshold trigger. Recording continues to flash capacity without a pulse (about 30 s measured with hardware timestamps). Defaults are 50 g and 10+3 s. Early pulses retain available history with a quality warning; capacity can truncate a late post-window. No flash erasure during acquisition. |
| F-06 | Persistent measurement after event | Dedicated internal-flash partition, CRC-protected commit written last, download after restart, explicit deletion before reuse. One shot retained at a time. Hard-reset persistence tested; physical power-cycle testing remains. |
| F-07 | Saturation, missing samples, sensor errors | Direct SPI/queue/data-ready interval errors (legacy FIFO/read/tag errors), storage errors, near-rail saturation, per-sensor sample indices, page sequences, header/data CRCs, window coverage warnings. A clear mask does not prove synchronization or detect every physical sample loss. Session quality flags are conservative and can include events before the retained window. |
| F-08 | Raw and processed export, CSV + binary + metadata | Original raw BVR file, NPZ with unchanged int16 raw counts plus calibrated arrays, CSV, JSON metadata through `shot_tool.py`. Invalid shots can be exported for diagnosis and remain labelled invalid. |
| F-09 | XYZ, resultant, spectrum, envelope | Unified phone page: XYZ with wheel/button/range zoom, pan and PNG export. PC `analyze`: XYZ, magnitude, Hann-window FFT, 25 ms RMS vibration envelope; existing interactive inspector retained. |
| F-10 | Shot/configuration comparison | PC `compare`: trigger-relative overlays and per-configuration repeated peak mean/std; flagged shots excluded from repeat statistics. |
| F-11 | Calibration/orientation checks | Documented stationary 10 s reference, offset calculation and explicit orientation matrices. Scale traceability, six-face verification and mechanical placement remain physical acceptance tasks. |
| F-12 | Unique shot ID and test metadata | 128-bit ID retained in flash; browser, Wi-Fi and USB arming accept test metadata. USB defaults to `source=USB`. Exports retain the ID and configuration. |

The recommended ≥100 valid shots per configuration are collected by exporting and then explicitly deleting each shot, preserving a PC/phone archive. Internal flash cannot retain 100 full-rate 13-second recordings at once. It has finite erase endurance and should not be operated as an indefinite unattended recorder.

The mechanical design, flex fabrication, sensor/cable masses, protected connectors, mounting stability, low-g handle channel, battery integration and ownership/delivery paperwork cannot be created or validated from this firmware repository. They remain customer acceptance items.
