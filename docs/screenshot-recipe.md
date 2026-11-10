# README screenshots

`mise run screenshots` runs `tools/screenshots.sh`, which regenerates `docs/screenshots/readme-preview.png` from the simulator. Run it after changing `src/page.html`.

```shell
mise run screenshots # needs port 8080 free, exits 1 if a sim is already running
```

## What it does

1. Builds the sim and starts it with `SIM_SETTINGS` pointing at a temp file, so `target/sim-settings.bin` is untouched
2. Seeds room 0 through the API: name "Living room", a convex six point outline, mode Random
3. Captures three 390x844 screens with `playwright-cli` into `docs/screenshots/screenshot-{random,manual,outline}.png`, after waiting for the first `/status` and any toast to clear
4. Serves the repo root on port 13381 and captures `tools/screenshot-sheet.html` with `omitBackground: true` for a transparent PNG

The trap kills the sim and the http server and closes the `playwright-cli` session on exit, including on failure. The Random status countdown and phase vary between runs.

## Sheet layout

- Canvas 1450x1000
- Phones 390x844, the capture size, so nothing is resampled
- Corner radius 34, gap 80, phones at x 60, 530, 1000 and y 50
- Shadow `dy=14 stdDeviation=18 flood-opacity=0.16`

Keep at least `dy + 3 * stdDeviation` (68 px) between the phones and the bottom edge, and `3 * stdDeviation` (54 px) on the sides, or the shadow gets cut by the canvas. With `omitBackground: true` the cut is not visible as a line, so after moving the phones check that the outermost rows and columns of the PNG have alpha 0.
