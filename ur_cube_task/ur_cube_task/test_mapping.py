def pixel_to_base(pixel_x, pixel_y):
    base_x_mm = 0.064948 * pixel_x + 0.611157 * pixel_y - 489.036
    base_y_mm = 0.657524 * pixel_x + 0.027756 * pixel_y - 316.835
    base_z_mm = 236

    return base_x_mm, base_y_mm, base_z_mm


points = {
    "red": (76, 407),
    "yellow": (302, 78),
    "blue": (586, 253),
}

for name, (x, y) in points.items():
    bx, by, bz = pixel_to_base(x, y)
    print(f"{name}: base=({bx:.1f}, {by:.1f}, {bz:.1f}) mm")

