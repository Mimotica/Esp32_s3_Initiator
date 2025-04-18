ESP32 FTM Time‑Sync Demo

This repo contains two ESP‑IDF projects demonstrating Wi‑Fi FTM time‐sync:

ftm_responder – Sniffs FTM bursts and computes sync‑offsets

ftm_initiator – Sends periodic FTM bursts and logs TX/RX timestamps


Repository structure

/
├── ftm_responder/
│   ├── main/
│   │   └── ftm_main.c
│   ├── CMakeLists.txt
│   └── menuconfig.defaults
├── ftm_initiator/
│   ├── main/
│   │   └── ftm_main.c
│   ├── CMakeLists.txt
│   └── menuconfig.defaults
└── README.md    ← you are here

Prerequisites
ESP‑IDF v5.4 (or later) installed & in PATH

Two ESP32‑S3 boards (one as Initiator, one as Responder)

A USB‑serial cable per board

Which log lines to watch

Responder

mathematica
Αντιγραφή
Επεξεργασία
I (…) FTM_RESPONDER: Responder ready; bursts expected every 1000000 us
I (…) FTM_RESPONDER: FTM burst start ts = <T1> us  
I (…) FTM_RESPONDER: Burst Δ = <Δ> us, Inst. Offset = <ins
                                                              
                                                              
Initiator

I (…) FTM_INITIATOR: TX at <TX_ts> us  
I (…) FTM_INITIATOR: RX at <RX_ts> us, Δ=<Δ> ust> us, Avg Offset = <avg> us  
I (…) FTM_RESPONDER: Session Sync Offset: <sync_offset> ns
