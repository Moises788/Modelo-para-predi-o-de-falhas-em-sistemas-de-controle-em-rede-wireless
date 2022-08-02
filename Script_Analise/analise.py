from turtle import color
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

#Leitura e conversão de tempo do dataframe de controle
data = pd.read_csv('E:\Bolsistas\Moisés\Codes\Analise_dados\dados.csv')
data['Date'] = pd.to_datetime(data['Date'])

tempo = data["Date"]
Nivel_tank = data["LV1"]
Saida = data["Output"]
Setpoint = data["SPVolt"]

erro = abs(Nivel_tank-Setpoint)

zeros = [0]*len(tempo)

erro = abs(Nivel_tank-Setpoint)

plt.scatter(tempo, Setpoint, label = 'Setpoint', s=1, color = 'red')
plt.scatter(tempo, Nivel_tank, label = 'Níve do tank', s=1, color = 'green')
plt.scatter(tempo, erro, label = 'erro', s=1)
plt.legend() #legenda do gráfico
plt.xlabel('Tempo')
plt.ylabel('cm')



plt.show()