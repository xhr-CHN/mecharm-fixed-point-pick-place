"""Create compact visual-QA sheets from rendered individual-report pages."""

from pathlib import Path

from PIL import Image, ImageDraw


PAGE_DIR = Path(__file__).resolve().parent / "tmp" / "pdfs-final"
pages = sorted(PAGE_DIR.glob("page-*.png"))

for group in range((len(pages) + 5) // 6):
    thumbnails = []
    for page in pages[group * 6 : (group + 1) * 6]:
        image = Image.open(page).convert("RGB")
        image.thumbnail((420, 594))
        canvas = Image.new("RGB", (440, 630), "white")
        canvas.paste(image, ((440 - image.width) // 2, 25))
        ImageDraw.Draw(canvas).text((10, 5), page.stem, fill="black")
        thumbnails.append(canvas)

    rows = (len(thumbnails) + 1) // 2
    sheet = Image.new("RGB", (880, 630 * rows), "#dddddd")
    for index, image in enumerate(thumbnails):
        sheet.paste(image, ((index % 2) * 440, (index // 2) * 630))
    sheet.save(PAGE_DIR / f"contact-{group + 1}.jpg", quality=88)
