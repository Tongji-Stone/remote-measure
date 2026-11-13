"""Generate an A4 landscape OpenCV chessboard calibration target.

The board has 9 columns x 7 rows of *inner corners*, hence 10 x 8 printed
black/white cells.  Each cell is exactly 20 mm in the PDF coordinate system.
"""

from pathlib import Path

from reportlab.lib.colors import black, white
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


INNER_CORNERS_X = 9
INNER_CORNERS_Y = 7
CELL_SIZE_MM = 20.0
PAGE_SIZE = landscape(A4)
OUTPUT = Path(__file__).with_name("棋盘格_9x7内角点_A4横向_20mm.pdf")


def main() -> None:
    page_width, page_height = PAGE_SIZE
    columns = INNER_CORNERS_X + 1
    rows = INNER_CORNERS_Y + 1
    cell = CELL_SIZE_MM * mm
    board_width = columns * cell
    board_height = rows * cell
    origin_x = (page_width - board_width) / 2
    origin_y = (page_height - board_height) / 2

    pdf = canvas.Canvas(str(OUTPUT), pagesize=PAGE_SIZE)
    pdf.setTitle("OpenCV chessboard - 9x7 inner corners - 20 mm cells")
    pdf.setAuthor("Camera calibration materials")
    pdf.setFillColor(white)
    pdf.rect(0, 0, page_width, page_height, stroke=0, fill=1)

    # The top-left cell is black.  Rows are indexed from the top so this is
    # visually unambiguous when comparing the printed sheet with this source.
    pdf.setFillColor(black)
    for row in range(rows):
        for col in range(columns):
            if (row + col) % 2 == 0:
                x = origin_x + col * cell
                y = origin_y + (rows - 1 - row) * cell
                pdf.rect(x, y, cell, cell, stroke=0, fill=1)

    pdf.showPage()
    pdf.save()
    print(f"Created: {OUTPUT}")
    print(f"Board: {columns} x {rows} cells; {INNER_CORNERS_X} x {INNER_CORNERS_Y} inner corners")
    print(f"Cell size: {CELL_SIZE_MM:.1f} mm; board size: {board_width / mm:.1f} x {board_height / mm:.1f} mm")


if __name__ == "__main__":
    main()
