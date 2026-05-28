import numpy as np


def calibrate(name, pixels, base):
    pixels = np.array(pixels, dtype=float)
    base = np.array(base, dtype=float)
    A = np.column_stack([pixels, np.ones(len(pixels))])
    coeffs_x, _, _, _ = np.linalg.lstsq(A, base[:, 0], rcond=None)
    coeffs_y, _, _, _ = np.linalg.lstsq(A, base[:, 1], rcond=None)
    print(f'=== {name} ===')
    print(f'base_x = {coeffs_x[0]:.8f} * px + {coeffs_x[1]:.8f} * py + {coeffs_x[2]:.8f}')
    print(f'base_y = {coeffs_y[0]:.8f} * px + {coeffs_y[1]:.8f} * py + {coeffs_y[2]:.8f}')
    print()


# Målepunkter fra simulering med fake_camera og testbilde.
# Pikselkoordinater fra /detected_cubes, robotkoordinater fra
# pixel_to_base_mm (hardkodet mapping fra tidlig utvikling).

# red:76,407   -> base (-235.4, -255.6) mm
# yellow:302,78 -> base (-421.8, -116.1) mm
# blue:586,253  -> base (-296.4,  75.5) mm
pixels_sim = [
    [76,  407],
    [302, 78],
    [586, 253],
]
base_sim = [
    [-235.4, -255.6],
    [-421.8, -116.1],
    [-296.4,   75.5],
]


# Målepunkter fra fysisk robot:

# Søkeposisjon 1
# red:198,85   -> TCP (0.4326, 0.0789)
# green:11,357 -> TCP (0.6443, -0.0556)
# blue:381,322 -> TCP (0.5631, 0.2493)
# green:397,392 -> TCP (0.6185, 0.2701)
# blue:330,139  -> TCP (0.4537, 0.1889)
pixels_search1 = [
    [198, 85],
    [11, 357],
    [381, 322],
    [397, 392],
    [330, 139],
]
base_search1 = [
    [0.4326,  0.0789],
    [0.6443, -0.0556],
    [0.5631,  0.2493],
    [0.6185,  0.2701],
    [0.4537,  0.1889],
]

# Søkeposisjon 2
# red:26,42    -> TCP (0.8891, -0.0800)
# green:384,49 -> TCP (0.8478,  0.2270)
# blue:250,217 -> TCP (0.9885,  0.1324)
# red:129,168  -> TCP (0.9706,  0.0246)
# green:407,251 -> TCP (0.9899, 0.2583)
# blue:300,46  -> TCP (0.8595,  0.1531)
pixels_search2 = [
    [26,  42],
    [384, 49],
    [250, 217],
    [129, 168],
    [407, 251],
    [300, 46],
]
base_search2 = [
    [0.8891, -0.0800],
    [0.8478,  0.2270],
    [0.9885,  0.1324],
    [0.9706,  0.0246],
    [0.9899,  0.2583],
    [0.8595,  0.1531],
]

calibrate('Søkeposisjon 1', pixels_search1, base_search1)
calibrate('Søkeposisjon 2', pixels_search2, base_search2)
