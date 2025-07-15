import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from util import Global
import numpy as np
import FindCarIndex
from engelkacis import escapeFromObstacle
def en_yakin_index(engel_index):
    # Mesafe hesabı için Öklid normunu kullanın
    mesafeler = np.linalg.norm(Global.corner_point - engel_index, axis=1)

    # En yakın noktanın indeksini bulun
    en_yakin_idx = np.argmin(mesafeler)

    # En yakın noktayı ve mesafeyi seçin
    en_yakin_nokta = Global.corner_point[en_yakin_idx]
    en_yakin_mesafe = mesafeler[en_yakin_idx]

    print(f"En yakın nokta: {en_yakin_nokta}")
    print(f"En yakın mesafe: {en_yakin_mesafe}")
    return en_yakin_nokta

def Girilmez():
    vehicle_x,vehicle_y= Global.vehicle_location
    vehicle_direction = Global.vehicle_direction
    merkez_nokta = Global.girilmez_location
    #2 nokta arasındaki uzaklıgı buluyoruz
    data=Global.csv_to_list()
    #burada index bulma kısmında sıkıntı yasanırsa aracın bulundugu indexten yukarı doğru ilk sola dönüş te kapatilabilir
    #yöne göre 1 metre çıkartıyorumki engelin dışarda olmasına önlem almış olayım
    #engelin koordinatı
    if vehicle_direction=="K":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = merkez_nokta[0]+vehicle_y
        print(engel_x,engel_y)

        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x-1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x-1][y]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="G":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = vehicle_y-merkez_nokta[0]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x+1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x+1][y]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="B":
        engel_x = vehicle_x-merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y-1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x][y-1]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="D":
        engel_x = vehicle_x+merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y+1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x+1][y]=0
        merkez_nokta=[engel_x,engel_y]  


def SagaDonulmez():
    vehicle_x,vehicle_y= Global.vehicle_location
    vehicle_direction = Global.vehicle_direction
    merkez_nokta = Global.saga_donulmez_location
    #2 nokta arasındaki uzaklıgı buluyoruz
    data=Global.csv_to_list()
    print(merkez_nokta)
    #burada index bulma kısmında sıkıntı yasanırsa aracın bulundugu indexten yukarı doğru ilk sola dönüş te kapatilabilir
    #yöne göre 1 metre çıkartıyorumki engelin dışarda olmasına önlem almış olayım
    #engelin koordinatı
    if vehicle_direction=="K":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = merkez_nokta[0]+vehicle_y
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x-1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x][y+1]=0

        merkez_nokta=[engel_x,engel_y]
        print("meeee",merkez_nokta)
        print(Global.matrix)
    elif vehicle_direction=="G":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = vehicle_y-merkez_nokta[0]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x+1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x][y-1]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="B":
        engel_x = vehicle_x-merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y-1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x-1][y]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="D":
        engel_x = vehicle_x+merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y+1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x+1][y]=0
        merkez_nokta=[engel_x,engel_y]  


def SolaDonulmez():
    vehicle_x,vehicle_y= Global.vehicle_location
    vehicle_direction = Global.vehicle_direction

    seritUzunlugu=0

    merkez_nokta = Global.sola_donulmez_location
    #2 nokta arasındaki uzaklıgı buluyoruz
    data=Global.csv_to_list()


    #burada index bulma kısmında sıkıntı yasanırsa aracın bulundugu indexten yukarı doğru ilk sola dönüş te kapatilabilir
    #yöne göre 1 metre çıkartıyorumki engelin dışarda olmasına önlem almış olayım
    #engelin koordinatı
    if vehicle_direction=="K":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = merkez_nokta[0]+vehicle_y
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x-1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x][y-1]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="G":
        engel_x = merkez_nokta[1]+vehicle_x
        engel_y = vehicle_y-merkez_nokta[0]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x+1, engel_y, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x][y+1]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="B":
        engel_x = vehicle_x-merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y-1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x+1][y]=0
        merkez_nokta=[engel_x,engel_y]
    elif vehicle_direction=="D":
        engel_x = vehicle_x+merkez_nokta[0]
        engel_y = vehicle_y+merkez_nokta[1]
        engel_index_x,engel_index_y=FindCarIndex.findcarindex(engel_x, engel_y+1, data)
        engel_index=np.array([engel_index_x, engel_index_y])
        x,y=en_yakin_index(engel_index)
        Global.matrix[x-1][y]=0
        merkez_nokta=[engel_x,engel_y]  


    #BURADA HESAPLANAN NERKEZ NOKTAYA GÖRE İNDEXİ KAPATMA İŞLEMİ YAPIYORIZ


   

        



