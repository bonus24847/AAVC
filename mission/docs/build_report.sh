#!/usr/bin/env bash
# Build the AAVC 2026 technical report: Markdown -> LaTeX -> PDF (XeLaTeX).
#
#   bash docs/build_report.sh
#
# Inputs : docs/AAVC2026_Technical_Report.md   (the source of truth)
#          docs/report_preamble.tex            (fonts/headers/table styling)
# Outputs: docs/AAVC2026_Technical_Report.tex  (standalone LaTeX, keepable)
#          docs/AAVC2026_Technical_Report.pdf
#
# The first 7 lines of the markdown are the title block; they become LaTeX
# title-page metadata instead of body text, and every remaining heading is
# promoted one level (## -> \section) so the numbering in the text matches the
# numbering in the table of contents.
set -euo pipefail
cd "$(dirname "$0")"

SRC=AAVC2026_Technical_Report.md
STEM=AAVC2026_Technical_Report
BODY=$(mktemp -t aavc_report_body_XXXXXX.md)
trap 'rm -f "$BODY"' EXIT
tail -n +8 "$SRC" > "$BODY"

pandoc "$BODY" \
  --standalone \
  --from=markdown \
  --toc --toc-depth=2 \
  --shift-heading-level-by=-1 \
  --include-in-header=report_preamble.tex \
  -V documentclass=article \
  -V classoption=titlepage \
  -V papersize=a4 \
  -V fontsize=10pt \
  -V geometry:margin=2.2cm \
  -V mainfont="TeX Gyre Termes" \
  -V sansfont="TeX Gyre Heros" \
  -V monofont="DejaVu Sans Mono" \
  -V monofontoptions="Scale=0.80" \
  -V title="AAVC 2026 — Technical Design \& Analysis Report" \
  -V subtitle="Autonomous PX4 Hexacopter for Precision Fragile-Cargo Delivery — EFT X6100 · Pixhawk 6X · Raspberry Pi CM4" \
  -V author="Team AeroOptix · KMUTNB, Faculty of Engineering" \
  -V date="Document version 2.0 · 28 August 2026 · Rules \& Regulations V1.3 as amended by the 24 Jul and 28 Aug 2026 event briefings" \
  -o "$STEM.tex"

latexmk -xelatex -interaction=nonstopmode -halt-on-error -quiet "$STEM.tex" >/dev/null
# Report anything the fonts could not set or the page could not hold, then clean.
MISSING=$(grep -c "Missing character" "$STEM.log" || true)
OVERFULL=$(grep -c "Overfull \\\\hbox" "$STEM.log" || true)
[ "$MISSING" = "0" ] || echo "WARNING: $MISSING missing glyph(s) — see $STEM.log"
[ "$OVERFULL" = "0" ] || echo "WARNING: $OVERFULL overfull box(es) — see $STEM.log"
latexmk -c "$STEM.tex" >/dev/null 2>&1 || true

echo "built: docs/$STEM.tex  ->  docs/$STEM.pdf ($(du -h "$STEM.pdf" | cut -f1), $(pdfinfo "$STEM.pdf" | awk '/^Pages/{print $2}') pages)"
