from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
from util import Global
import Path
import FindCarIndex
from NesneTespit import buyukDuba,Donulmez

#bu değerler aracın anlık değerleri sensorlerden gelecek
vehicle_x,vehicle_y=Global.vehicle_location
x_main=[vehicle_x]
y_main=[vehicle_y]

data=Global.csv_to_list()

for i in range(1,3):
    #araç başlangıçta tek sferlik bir rota oluşturmalı, eğer oluşturulan rotanın üzerinde engel v.b şey var ise tetikleniniğ tekrar çalıştırlmalı
    if Global.firstTime:
        for mission in Global.mission:
            vehicle_x,vehicle_y = x_main[-1],y_main[-1]
            target_index_x, target_index_y = mission # görev indexlerini alıyoruz
            x_main,y_main= Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)
        Global.firstTime=False
        Global.x_main=x_main
        Global.y_main=y_main

    else:
        x_main=[vehicle_x]
        y_main=[vehicle_y]
        #DUBA tetiklendiğinde(10 metre, 5metre neyse) rota oluşacak, bir yerden main.py çağırılmalı. çağrıldıgında global iiçndeki engel bilgisi verilmeli. sonrası okay
        if len(Global.dubaEngel)>0 :
            x_main,y_main = buyukDuba.dubaEngel()


        
        #sola ve saga donulmezler
        if Global.sola_donulmez:
            Donulmez.SolaDonulmez()
            Global.sola_donulmez=False
        if Global.saga_donulmez:
            Donulmez.SagaDonulmez()()
            Global.saga_donulmez=False
        if Global.stop:
            Donulmez.Stop()
            Global.stop=False
        print(Global.matrix)
        for mission in Global.mission:
            vehicle_x,vehicle_y = x_main[-1],y_main[-1]
            target_index_x, target_index_y = mission # görev indexlerini alıyoruz
            x_main,y_main= Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)

        Global.x_main=x_main
        Global.y_main=y_main


# Plot the data
plt.plot(x_main, y_main, marker='o', linestyle='-')

# Set plot title and axis labels
plt.title('Rota')
plt.xlabel('X Ekseni')
plt.ylabel('Y Ekseni')
# Eksen sınırlarını manuel olarak belirleme
plt.xlim(-70, 10)
plt.ylim(-10, 70)
# Display the grid
plt.grid(True)

# Show the plot
plt.show()
