# FEAD Belt Drive System — Technical Analysis Web Application

> **Front End Accessory Drive (FEAD) engineering analysis tool for the Ashok Leyland H6 engine with Gates MT820 8PK AMD Aramid belt.**

[![GitHub Pages](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-blue?logo=github)](https://YOUR-USERNAME.github.io/fead-belt-drive-system/)

---

## 🔬 What This Is

A complete browser-based **FEAD engineering analysis website** that computes in real-time:

| Feature | Details |
|---|---|
| **Belt Slip Analysis** | Eytelwein / Capstan equation per pulley — slip safety factor (SF) |
| **Hub Load Computation** | Vector-sum tangent geometry for all 6 pulleys vs Gates PDF limits |
| **FEAD Efficiency** | Bending, bearing, slip, and centrifugal losses — η in % |
| **Tension Optimiser** | Minimum static pre-load for SF ≥ 1.3 at any RPM |
| **Drive Cycle Analysis** | WLTC vs NEDC worst-case hub loads and SF margins |
| **2-D Live Canvas** | Interactive belt geometry diagram — updates with every slider |
| **3-D Model** | Three.js rotating FEAD model with force arrows |
| **Compliance Dashboard** | Radar chart, score ring, maintenance schedule |
| **PDF Report** | 8-page professional engineering report (jsPDF, auto-generated) |

### Data Sources
- Gates Accessory Belt Drive PDF — DB Ver 2.45.0.0
- C&U Group WR25153 fan bearing calculation — QZ20230803
- ISO 9981:2018 (belt slip safety)

---

## 🚀 Quick Start — Run Locally in 30 Seconds

### Option A — Just open the file (simplest)
```
1. Download or clone this repo
2. Double-click  index.html
3. Done — no server, no install needed
```

### Option B — Local dev server (recommended for full features)

**Prerequisites:** [Node.js](https://nodejs.org) installed

```bash
# Install once
npm install -g serve

# Run from project folder
serve .

# Open in browser:  http://localhost:3000
```

**Or with Python (if you have Python installed):**
```bash
# Python 3
python -m http.server 8080

# Open:  http://localhost:8080
```

---

## 📁 Project Structure

```
fead-belt-drive-system/
├── index.html          # Main page — all sections
├── index.css           # Dark enterprise design system
├── app.js              # Physics engine + charts + PDF generator
└── README.md           # This file
```

**No build step. No framework. No dependencies to install.**  
All libraries load from CDN (Chart.js, Three.js, jsPDF, SheetJS).

---

## 🌐 Live on GitHub Pages

This site is deployed via GitHub Pages.  
→ **[Open Live App](https://YOUR-USERNAME.github.io/fead-belt-drive-system/)**

---

## ⚙️ Physics Models

```
Belt geometry    →  Tangent-circle → span angles → hub load vectors
Hub loads        →  F_hub = √(T_in² + T_out² − 2·T_in·T_out·cos(π−θ))
Capstan eq.      →  T_tight / T_slack = e^(μθ)
Slip SF          →  ln(T_t / T_s) / (μθ)
Bending loss     →  C_b · (T_avg / R) · v  per pulley
Bearing loss     →  μ_b · F_hub · ω · R    per bearing
Centrifugal      →  T_c = m_b · v² / L
FEAD efficiency  →  η = P_accessories / (P_accessories + P_losses)
Bearing L10      →  L10 ∝ (C / F)³  (ISO 281)
```

---

## 🖥️ Browser Support

Works in all modern browsers: Chrome, Edge, Firefox, Safari.  
No Internet Explorer support (uses ES6+, Canvas, WebGL).

---

## 📄 License

MIT — free to use, modify, and distribute.
