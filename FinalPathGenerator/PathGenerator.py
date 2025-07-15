from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
from FinalPathGenerator.util import Global
from FinalPathGenerator import Path
import FinalPathGenerator.FindCarIndex
from FinalPathGenerator.NesneTespit import buyukDuba,Donulmez
import Main

#First Time
#Büyük Duba Olması
#Sola Donulmez
#Sağa Donulmez
#Girilmez

def get_firstTrajectory():
    x_main = []
    y_main = []
    x_main,y_main= Main.getFinalTrajectory()
    return x_main,y_main

def dubaEscape():
    x_main = []
    y_main = []
    x_main,y_main= Main.escapeObstacle()
    return x_main,y_main


#dubada 2 şerit kapanıyorsa matrixten orası kapatılmalı aksi takdirde matrixten kalıcı değişilik yapılmamalı ona ayarla
#bu hali ile patth generator stabil bir şekiledeçalışmakta

# #sensorden duba bilgisi geldiğinde dubanın lokasyon bilgisi ve dubaVar bilgisi Global.py içind düzenlenecek zaten. buradan sadece escape obstacle çağırmamaız yeter
# if Global.dubaVar:
#     x_main,y_main= Main.escapeObstacle()
# elif Global.sola_donulmez:
#     x_main,y_main= Main.solaDonulmez()
# elif Global.saga_donulmez:
#     x_main,y_main= Main.sagaDonulmez()
# elif Global.girilmez:
#     x_main,y_main= Main.girilmez()



# # Plot the data
# plt.plot(x_main, y_main, marker='o', linestyle='-')

# # Set plot title and axis labels
# plt.title('Rotaa')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# # Eksen sınırlarını manuel olarak belirleme
# plt.xlim(-70, 10)
# plt.ylim(-10, 70)
# # Display the grid
# plt.grid(True)

# # Show the plot
# plt.show()