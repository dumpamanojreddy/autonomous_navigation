#!/usr/bin/env python

import functools as ft
import copy

# Library implementing Bresenham line tracing
# From Wikipedia: https://en.wikipedia.org/wiki/Bresenham's_line_algorithm

# ##### Type Definitions #####
# GridCell := (int, int)
# NormalizedGridCell := (int, int)
# Octant := int  in range [0, 8]

# "denormalized" cell coordinate form (GridCell) is:
# 1. Absolute
# 2. In any octant
# "normalized" cell coordinate form (NormalizedGridCell) is:
# 1. Relative to origin
# 2. In octant 0


# ##### Algorithm #####

# line: GridCell GridCell int int -> Listof(GridCell)
# extension = number of cells to extend line beyond end cell
def line(origin, cell1, extension = 0, thickness = 0):
    cell1_octant = octant(origin, cell1)
    line = None
    if cell1_octant is "vertical":
        line = vertical_cells(origin, cell1, extension)
    else:
        line = line_cells(origin, cell1,
                          ft.partial(normalize_from, cell1_octant, origin),
                          ft.partial(denormalize_to, cell1_octant, origin),
                          extension)
    return thicken(line, thickness)

# vertical_cells: GridCell GridCell int -> Listof(GridCell)
def vertical_cells(origin, cell1, extension):
    (xO, yO) = origin
    (x1, y1) = cell1

    if (yO > y1):
        return vertical_cells(cell1, origin, extension)
    
    return map(lambda y: (xO, y),
               range(yO, y1 + 1 + extension))

# vertical_cells:
#  GridCell
#  GridCell
#  (GridCell -> NormalizedGridCell)
#  (NormalizedGridCell -> GridCell)
#  int
#  -> Listof(GridCell)
def line_cells(origin, cell1, normalizer, denormalizer, extension):
    cells = []
    # (x0, y0) = origin
    (x0, y0) = (0, 0)
    (x1, y1) = normalizer(cell1)
    (dx, dy) = (x1 - x0, y1 - y0)
    D = 2*dy - dx
    y = y0
    for x in range(x0, x1 + 1 + extension):
        cells.append(denormalizer((x, y)))
        if D > 0:
            y += 1
            D -= 2*dx
        
        D += 2*dy

    return cells


# ##### Helper Functions #####

# thicken: Listof(GridCell) int [bool] -> Listof(GridCell)
def thicken(line, thickness, left=True):
    if thickness > 0:
        new_line = copy.deepcopy(line)
        for cell in line:
            (x, y) = cell
            new_cell = (x + (-1 if left else 1), y)
            if new_cell not in new_line:
                new_line.append(new_cell)

        return thicken(new_line, thickness - 1, not left)

    else:
        return line

# octant: GridCell GridCell -> Octant
# Get the octant in which the line from ORIGIN to CELL1 lies
# Courtesy of Wikipedia (where the quadrant dividing lines have slope = +/- 1),
# Octants:
#   \2|1/
#   3\|/0
#  ---+---
#   4/|\7
#   /5|6\
def octant(origin, cell1):
    (xO, yO) = origin
    (x1, y1) = cell1
    (dx, dy) = (x1 - xO, y1 - yO)
    if dx == 0:
        return "vertical"
    slope = float(dy)/float(dx)
    if dx > 0:
        if dy > 0:
            return 0 if slope <= 1 else 1
        else:
            return 7 if -slope <= 1 else 6
    else:
        if dy > 0:
            return 3 if -slope <= 1 else 2
        else:
            return 4 if slope <= 1 else 5


# normalize_from: Octant GridCell GridCell -> NormalizedGridCell
# Converts a "denormalized" (absolute, any-octant) cell into
# a "normalized" (relative to ORIGIN, octant 0) cell
def normalize_from(octant, origin, denormalized_absolute_cell):
    assert(0 <= octant and octant < 8)

    (oX, oY) = origin
    (absX, absY) = denormalized_absolute_cell
    (relX, relY) = (absX - oX, absY - oY)
    if   octant == 0:
        return (relX, relY)
    elif octant == 1:
        return (relY, relX)
    elif octant == 2:
        return (relY, -relX)
    elif octant == 3:
        return (-relX, relY)
    elif octant == 4:
        return (-relX, -relY)
    elif octant == 5:
        return (-relY, -relX)
    elif octant == 6:
        return (-relY, relX)
    elif octant == 7:
        return (relX, -relY)

# denormalize_to: Octant GridCell NormalizedGridCell -> GridCell
# Converts a "normalized" (relative to ORIGIN, octant 0) cell into
# a "denormalized" (absolute, any-octant) cell
def denormalize_to(octant, origin, normalized_relative_cell):
    assert(0 <= octant and octant < 8)
   
    (oX, oY) = origin
    (relX, relY) = normalized_relative_cell
    (absX, absY) = (oX + relX, oY + relY)
    if   octant == 0:
        return (oX + relX, oY + relY)
    elif octant == 1:
        return (oX + relY, oY + relX)
    elif octant == 2:
        return (oX + -relY, oY + relX)
    elif octant == 3:
        return (oX + -relX, oY + relY)
    elif octant == 4:
        return (oX + -relX, oY + -relY)
    elif octant == 5:
        return (oX + -relY, oY + -relX)
    elif octant == 6:
        return (oX + relY, oY + -relX)
    elif octant == 7:
        return (oX + relX, oY + -relY)
