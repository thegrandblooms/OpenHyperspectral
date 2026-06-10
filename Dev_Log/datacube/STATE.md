# Datacube Module — Current State & Plan

**Last updated:** 2026-03-27
**Status:** 🔴 NOT STARTED — depends on Scan module output

---

## What This Module Does

Takes a completed scan session (video + encoder log) and assembles a hyperspectral datacube.

**This module sits between:**
- ⬅ Scan module (`scans/<session>/video.avi` + `encoder_log.csv` + `metadata.json`) — inputs
- ➡ Analysis / visualization — outputs

---

## Inputs (from Scan module)

```
scans/<session_id>/
  video.avi               raw video (all frames, Mightex native)
  encoder_log.csv         esp_ms, pos_deg, vel_deg_s, target_deg, vq, elec_deg, os_time_ms
  metadata.json           session params, clock offset, sweep config
  frames/                 (pre-extracted by frame_extractor.py)
    frame_0000.tiff       extracted at 0.00°
    frame_0001.tiff       extracted at 0.08°/pixel
    ...
```

---

## Outputs

```
datacubes/<session_id>/
  datacube.npy            3D array: [spatial_x, spatial_y, wavelength]
  wavelength_map.csv      pixel_index → wavelength_nm (from calibration)
  preview.png             false-color preview image
  metadata.json           inherits scan metadata + calibration info
```

---

## Architecture (TBD)

### Datacube assembly
Each extracted frame is a 2D image (752×480). The pushbroom line-scan axis maps to
angular position (one column per 0.08° = one wavelength). The spatial axis is the
perpendicular dimension.

Datacube dimensions:
- X (spatial): 480 pixels (perpendicular to sweep direction)
- Y (angular/spectral): 752 frames × 1 column each → 752 wavelength bins
- The exact spatial→wavelength mapping depends on spectrometer optics calibration

### Calibration
Wavelength calibration: known spectral lines (e.g., Hg lamp) → pixel → wavelength mapping.
Flat-field correction: uniform illumination scan → normalize per-pixel sensitivity.
Dark current: dark frame subtraction.

---

## Dependencies / Open Questions

- **Spectrometer optics:** What is the actual wavelength range and dispersion?
  This determines how angular position maps to wavelength.
- **Frame format:** Mightex frames are 8-bit grayscale. May need 16-bit for dynamic range.
- **Datacube format:** `.npy` is simple; ENVI `.hdr` + `.raw` is the hyperspectral standard
  and opens in tools like QGIS, spectral python, etc.
- **Spatial calibration:** Does the 480px height correspond to a known spatial scale?

---

## Build Order (future)

1. **Wavelength calibration script** — maps pixel column to wavelength from known source
2. **Datacube assembler** — stacks `frames/` into 3D array, applies wavelength map
3. **Preview generator** — false-color RGB from 3 bands
4. **ENVI export** — standard hyperspectral format for interop with analysis tools
