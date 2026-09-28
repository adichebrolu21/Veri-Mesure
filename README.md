<div align="center">

# Veri-Mesure

### Automated Garment Measurement & Quality Inspection

<img src="https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square&logo=cplusplus&logoColor=white"/>
<img src="https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?style=flat-square&logo=opencv&logoColor=white"/>
<img src="https://img.shields.io/badge/CMake-Build%20System-064F8C?style=flat-square&logo=cmake&logoColor=white"/>
<img src="https://img.shields.io/badge/Status-Active-111111?style=flat-square"/>

<br><br>

<i>Computer vision for turning garment images into precise, structured measurements.</i>

</div>

---

## ✦ About

**Veri-Mesure** is a modular computer-vision system built for automated
garment measurement and quality inspection.

It handles camera acquisition, calibration, segmentation, geometric
feature extraction, measurement, and QC validation.

---

## ✦ Capabilities

- 📷 Camera acquisition & calibration
- 🎯 ROI and garment presence detection
- ✂️ Garment segmentation
- 📐 Dimensional measurement
- ◌ Perimeter & curve analysis
- 👕 Collar detection
- 🔍 Edge & boundary processing
- ✓ Quality-control validation
- 🧪 Component-level testing
- 🗂️ Template-based garment configuration

---

## ✦ Tech Stack

`C++` · `OpenCV` · `CMake` · `JSON` · `SVG`

Built with a modular architecture where individual vision and
measurement components can be developed, tested, and debugged independently.

---

## ✦ Structure

```text
apps/        → Application entry points
src/         → Core implementation
include/     → C++ interfaces
tests/       → Validation & testing
templates/   → Garment configurations
docs/        → Technical documentation
