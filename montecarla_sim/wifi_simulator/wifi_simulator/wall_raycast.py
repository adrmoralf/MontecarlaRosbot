"""
Bresenham 2D raycast sobre OccupancyGrid.
Cuenta transiciones libre→ocupado en la línea robot→AP.
Desacoplado de Gazebo: usa el mapa SLAM, que es el mapa imperfecto que
el robot realmente tiene (honesto respecto al filtro).
"""
import math


def _bresenham(x0, y0, x1, y1):
    """Devuelve lista de (col, fila) en la línea discreta de (x0,y0) a (x1,y1)."""
    celdas = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        celdas.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * error
        if e2 > -dy:
            error -= dy
            x0 += sx
        if e2 < dx:
            error += dx
            y0 += sy
    return celdas


def count_walls(p_robot, p_ap, grid_data, grid_info, occupied_threshold=50):
    """
    Cuenta el número de paredes (transiciones libre→ocupado) en la línea
    que une p_robot con p_ap sobre el OccupancyGrid.

    Parámetros:
        p_robot: (x, y) en coordenadas world (metros)
        p_ap:    (x, y) en coordenadas world (metros)
        grid_data: list/array de int8, orden row-major (de nav_msgs/OccupancyGrid)
                   -1 = desconocido, 0 = libre, 100 = ocupado
        grid_info: nav_msgs/MapMetaData (resolution, origin, width, height)
        occupied_threshold: umbral para considerar una celda ocupada (default 50)

    Devuelve:
        int: número de transiciones libre→ocupado
    """
    resolucion = grid_info.resolution
    ox = grid_info.origin.position.x
    oy = grid_info.origin.position.y
    ancho = grid_info.width
    alto = grid_info.height

    col_robot = int((p_robot[0] - ox) / resolucion)
    fila_robot = int((p_robot[1] - oy) / resolucion)
    col_ap = int((p_ap[0] - ox) / resolucion)
    fila_ap = int((p_ap[1] - oy) / resolucion)

    celdas = _bresenham(col_robot, fila_robot, col_ap, fila_ap)

    paredes = 0
    anterior_libre = True
    for (col, fila) in celdas:
        if 0 <= fila < alto and 0 <= col < ancho:
            valor = grid_data[fila * ancho + col]
            ocupada = (valor > occupied_threshold)
        else:
            ocupada = False   # fuera de bounds → tratar como libre

        if ocupada and anterior_libre:
            paredes += 1
        anterior_libre = not ocupada

    return paredes
