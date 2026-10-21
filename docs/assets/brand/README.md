<a id="top"></a>
<p align="right">
  <img src="logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Assets — Brand

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../diagrams/README.md"><b>Forward &#8594;</b></a>
</p>

The Team Positronic brand artwork: the raster logo exports used across the documentation, and the palette variants created so the mark reads on **both light and dark
backgrounds**. Every page under `docs/` embeds the logo at the top with
`<img src="../../assets/brand/logo.png" width="460">`.

## Files

| File | Size | Format | Purpose |
| :--- | :--- | :--- | :--- |
| `logo.png` | 1600 × 533 | PNG (RGBA) | **Primary logo** — transparent background, embedded by every documentation page. |
| `logo_white.png` | 1600 × 533 | PNG (RGBA) | Original artwork: cyan-to-blue icon + **white wordmark**, for dark backgrounds. |
| `logo_v1_teal_indigo.png` … `logo_v5_fuchsia_violet.png` | 1600 × 533 | PNG (RGBA) | Palette variants (below), each re-tested for contrast on white **and** black. |
| `logo_variants_preview.png` | 1150 × 600 | PNG (RGB) | Comparison sheet: the palette variants on a split white / black background. |

All PNG exports share the **same alpha channel** as `logo_white.png`, so the geometry (icon, hexagon
ring, wordmark) is identical across every file — only the colours differ.

## Palette variants

Each variant is a horizontal gradient applied through the mark. They were chosen so the mark keeps a
contrast ratio of roughly **3.7:1 or better on both a white and a black background** (colour alone, no
outline), which is what makes a single file usable on either.

| File | Gradient (hex) |
| :--- | :--- |
| `logo_v1_teal_indigo.png` | `#0D9488` → `#0284C7` → `#4F46E5` → `#7C3AED` |
| `logo_v2_cyan_blue.png` | `#0891B2` → `#2563EB` → `#4338CA` |
| `logo_v3_teal_green.png` | `#0F766E` → `#0D9488` → `#16A34A` → `#059669` |
| `logo_v4_orange_rose.png` | `#EA580C` → `#DC2626` → `#E11D48` |
| `logo_v5_fuchsia_violet.png` | `#C026D3` → `#9333EA` → `#7C3AED` |

## Usage

Reference by relative depth — a page in `00_specs/hardware/` reaches the logo with
`../../assets/brand/logo.png`, a page in `docs/` with `assets/brand/logo.png`:

```html
<img src="../../assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
```

- Use `logo.png` on the default (light) documentation theme.
- Use `logo_white.png` on a dark background (its wordmark is white).
- Use one of the `logo_v*.png` variants when a page or deck needs a specific accent.

See the parent [`assets/README.md`](../README.md) for how images are organised across the tree.

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
