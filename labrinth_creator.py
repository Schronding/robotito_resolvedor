laberinto_str = [
    "#####E#",
    "# # # #",
    "#S# # #",
    "#   # #",
    "#######"
]

laberinto_real = [

        "############",
        "S    #     #",
        "####     ###",
        "#          #",
        "# # ####   #",
        "# #   #  ###",
        "#   #      #",
        "# #######  #",
        "#          #",
        "##########E#"] 

# También podemos definirlo con números para facilitar el procesamiento
# 0: Camino, 1: Muro, 2: Inicio, 3: Fin
WALL = 1
PATH = 0
START = 2
END = 3

def convertir_laberinto(laberinto_str):
    mapa_numerico = []
    pos_inicio = None
    pos_fin = None
    for r, fila_str in enumerate(laberinto_str):
        fila_num = []
        for c, char in enumerate(fila_str):
            if char == '#':
                fila_num.append(WALL)
            elif char == ' ':
                fila_num.append(PATH)
            elif char == 'S':
                fila_num.append(START)
                pos_inicio = (r, c)
            elif char == 'E':
                fila_num.append(END)
                pos_fin = (r, c)
        mapa_numerico.append(fila_num)
    if not pos_inicio or not pos_fin:
        raise ValueError("El laberinto debe tener un punto de Inicio (S) y Fin (E)")
    return mapa_numerico, pos_inicio, pos_fin

# Uso:
#laberinto_numerico, pos_inicio, pos_fin = convertir_laberinto(laberinto_str)
#print(laberinto_numerico)
#print("Inicio:", pos_inicio, "Fin:", pos_fin)

laberinto_numerico, pos_inicio, pos_fin = convertir_laberinto(laberinto_real)
print(laberinto_numerico)
print("Inicio:", pos_inicio, "Fin:", pos_fin)
