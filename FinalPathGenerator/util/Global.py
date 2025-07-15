import numpy as np
import csv
matrix = np.array([
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    ])




corner_point=([0,0],[0,3],[0,8],[0,11],[3,0],[3,3],[3,8],[3,11],[7,0],[7,3],[7,8],[7,11],[11,0],[11,3],[11,6],[11,8],[11,11])
#araç başlangıçta tek sferlik bir rota oluşturmalı, eğer oluşturulan rotanın üzerinde engel v.b şey var ise tetikleniniğ tekrar çalıştırlmalı
firstTime=True
x_main,y_main=[],[]

sola_donulmez=False
sola_donulmez_location=[10,2,1]
saga_donulmez=False
saga_donulmez_location=[10,1,1]
girilmez=False
girilmez_location=[10,2,1]

vehicle_location=[-0.0, 20.0]   #aracın anlık konumu
mission=[[0,5],[13,6]]

# bu yön bilgisi aracın sensorunden gelecek anlık olarak ve bu değere eşitlenecek
vehicle_direction="K"

# #tek duba oldugunu varsayıyoruz.  Engel seyini test edeceksen burayı yorumdan çıkar
# #lidardan Dubanın sağ ve sol değerlerini aldık. başta boş bir liste olacak, eğer engel olursa aşağıdaki gibi lidar noktaları gelecek
dubaVar=True
dubaEngel=([10,-1.5,0.3],[10,1.5,0.3])
# dubaEngel=()


# mission=[(0,5),(7,1),(13,6)]
# mission=[[0,5],[7,1],[13,6]]
# mission=[[1,11],[0,5],[7,1]]
# mission=[[0,5],[5,3],[13,6]]

def csv_to_list():
    file_path = 'FinalPathGenerator/util/waypoint.csv'
    data_list = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)

        for row in csv_reader:
            data_list.append(row)
    return data_list


# CSV dosyasının yolu
# Her satırı liste olarak elde et

# mission=[[0,5],[7,1],[13,6]] 13,6,-4,0,-30,-33.3,-5,0,-28,-35 


