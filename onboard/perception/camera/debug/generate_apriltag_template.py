"""Generate print-ready AprilTag templates on an A4 page.

By default the script writes a PDF for reliable physical sizing. PNG output is
still available when an image preview is useful.

Examples:
    # Default parameters: --family tag36h11 --tag-id 0 --tag-size-mm 120
    # Output: apriltag_tag36h11_id0_120mm_a4.pdf
    python onboard/perception/camera/debug/generate_apriltag_template.py \
        --family tag36h11 --tag-id 0 --tag-size-mm 120
    python onboard/perception/camera/debug/generate_apriltag_template.py \
        --family tag36h11 --tag-id 5 --tag-size-mm 120
    python onboard/perception/camera/debug/generate_apriltag_template.py \
        --family tag25h9 --tag-id 3 --tag-size-mm 120

    # Optional: also write a PNG preview with embedded DPI metadata.
    python onboard/perception/camera/debug/generate_apriltag_template.py \
        --family tag36h11 --tag-id 0 --tag-size-mm 120 --also-png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


FAMILY_TO_DICT = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def _mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def _build_marker(family: str, tag_id: int, side_px: int) -> np.ndarray:
    if family not in FAMILY_TO_DICT:
        raise ValueError(
            f"Unsupported family '{family}'. Choose one of: {sorted(FAMILY_TO_DICT)}"
        )
    marker_dict = cv2.aruco.getPredefinedDictionary(FAMILY_TO_DICT[family])
    marker = np.zeros((side_px, side_px), dtype=np.uint8)
    # OpenCV generates the square marker image directly; the caller is
    # responsible for printing it at 100% scale so the physical edge matches
    # --tag-size used by target_ball_detector.py.
    cv2.aruco.generateImageMarker(marker_dict, int(tag_id), side_px, marker, 1)
    return marker


def _draw_text_center(page: np.ndarray, text: str, y: int, scale: float, thickness: int = 2):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (w, h), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(0, (page.shape[1] - w) // 2)
    cv2.putText(page, text, (x, y + h), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return y + h + baseline


def _save_page(page: np.ndarray, output_path: Path, dpi: int) -> None:
    pil_image = Image.fromarray(cv2.cvtColor(page, cv2.COLOR_BGR2RGB))
    suffix = output_path.suffix.lower()

    if suffix == ".png":
        # Embed DPI metadata so print dialogs have an explicit physical size hint.
        pil_image.save(output_path, dpi=(dpi, dpi))
        return

    if suffix == ".pdf":
        pil_image.save(output_path, resolution=float(dpi))
        return

    raise ValueError(f"Unsupported output format '{suffix}'. Use .png or .pdf.")


def _default_output_path(script_path: Path, family: str, tag_id: int, tag_size_mm: float) -> Path:
    output_name = f"apriltag_{family}_id{tag_id}_{int(round(tag_size_mm))}mm_a4.pdf"
    return script_path.resolve().parent / output_name


def main():
    parser = argparse.ArgumentParser(description="Generate a printable A4 AprilTag template.")
    parser.add_argument("--family", default="tag36h11", help="AprilTag family, e.g. tag36h11.")
    parser.add_argument("--tag-id", type=int, default=0, help="AprilTag id to print.")
    parser.add_argument("--tag-size-mm", type=float, default=120.0,
                        help="Tag edge length in millimetres. This is the black square size.")
    parser.add_argument("--dpi", type=int, default=300, help="Output DPI.")
    parser.add_argument("--margin-mm", type=float, default=15.0, help="Minimum page margin in millimetres.")
    parser.add_argument(
        "--output",
        default=None,
        help="Output .pdf or .png path. Defaults to a sibling PDF for reliable printing.",
    )
    parser.add_argument(
        "--also-pdf",
        action="store_true",
        help="Also write a sibling PDF with the same physical page size.",
    )
    parser.add_argument(
        "--also-png",
        action="store_true",
        help="Also write a sibling PNG with embedded DPI metadata.",
    )
    args = parser.parse_args()

    family = str(args.family).strip().lower()
    dpi = int(args.dpi)
    page_w = _mm_to_px(210.0, dpi)
    page_h = _mm_to_px(297.0, dpi)
    margin_px = _mm_to_px(args.margin_mm, dpi)
    tag_side_px = _mm_to_px(args.tag_size_mm, dpi)

    if tag_side_px <= 0:
        raise ValueError("--tag-size-mm must be positive.")
    if tag_side_px + 2 * margin_px > min(page_w, page_h):
        raise ValueError("Tag is too large for the requested A4 layout and margin.")

    page = np.full((page_h, page_w, 3), 255, dtype=np.uint8)
    marker = _build_marker(family, args.tag_id, tag_side_px)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    top_y = _mm_to_px(18.0, dpi)
    next_y = _draw_text_center(page, "AprilTag Printable Template", top_y, scale=0.9, thickness=2)
    next_y += _mm_to_px(3.0, dpi)
    next_y = _draw_text_center(
        page,
        f"family={family}   id={args.tag_id}   tag_size={args.tag_size_mm:.1f} mm",
        next_y,
        scale=0.65,
        thickness=2,
    )
    next_y += _mm_to_px(6.0, dpi)

    x0 = (page_w - tag_side_px) // 2
    y0 = max(next_y, (page_h - tag_side_px) // 2 - _mm_to_px(8.0, dpi))
    y0 = min(y0, page_h - margin_px - tag_side_px - _mm_to_px(35.0, dpi))
    page[y0:y0 + tag_side_px, x0:x0 + tag_side_px] = marker_bgr

    guide_y = y0 + tag_side_px + _mm_to_px(10.0, dpi)
    guide_y = _draw_text_center(
        page,
        "Measure the black square edge only. Use that value for --tag-size.",
        guide_y,
        scale=0.6,
        thickness=2,
    )
    guide_y += _mm_to_px(2.0, dpi)
    _draw_text_center(
        page,
        "Print at 100% scale on A4. Do not fit-to-page.",
        guide_y,
        scale=0.6,
        thickness=2,
    )

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = _default_output_path(Path(__file__), family, args.tag_id, args.tag_size_mm)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    _save_page(page, output_path, dpi)
    print(f"[INFO] Wrote printable template to: {output_path}")

    if args.also_pdf and output_path.suffix.lower() != ".pdf":
        pdf_path = output_path.with_suffix(".pdf")
        _save_page(page, pdf_path, dpi)
        print(f"[INFO] Wrote printable PDF to: {pdf_path}")

    if args.also_png and output_path.suffix.lower() != ".png":
        png_path = output_path.with_suffix(".png")
        _save_page(page, png_path, dpi)
        print(f"[INFO] Wrote preview PNG to: {png_path}")


if __name__ == "__main__":
    main()
