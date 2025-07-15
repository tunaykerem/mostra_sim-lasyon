from util import Global as g
from FinalPathGenerator.FindCarIndex import findcarindex
from FinalPathGenerator.DurLevha import dur
from FinalPathGenerator.GirilmezLevha import girilmez
from FinalPathGenerator.SolaDonulmezLevha import solaDonulmez
from FinalPathGenerator.SagaDonulmezLevha import sagaDdonulmez

from matplotlib import pyplot as plt
from Util import Global as Global
def set_label (levhAdi):

    current_x,current_y=Global.current_x,Global.current_y
    direction=Global.direction
    carIndex=findcarindex(current_x,current_y, g.csv_to_list())

    if levhAdi == ['dur']:
        dur(carIndex,direction)

    elif levhAdi == ['giris_olmayan_yol']:
        girilmez(carIndex,direction)

    elif levhAdi == ['sola_donulmez']:
        solaDonulmez(carIndex,direction,current_x,current_y)

    elif levhAdi == ['saga_donulmez']:
        sagaDdonulmez(carIndex,direction,current_x,current_y)

# plt.plot(g.x_main, g.y_main, marker='o', linestyle='-')
# plt.title('Rotaa')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.xlim(-90, 10)
# plt.ylim(-10, 90)
# plt.grid(True)
# plt.show()

