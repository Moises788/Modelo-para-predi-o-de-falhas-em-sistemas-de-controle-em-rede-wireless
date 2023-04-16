# LAMP-IC-UFRN
## Esta documentação visa explicar de uma maneira mais amigável o desenvolvimento e resultados da minha bolsa de iniciação científica no LAMP-UFRN.

> Status: Developing ⚠️


### Objetivo do projeto

Desenvolver um modelo de rede neural para predição do erro de controle a partir da qualidade da rede sem fio dos equipamentos presentes em um processo industrial.

### Principais Tecnologias Utilizados

* Google Colab
* Python
* TensorFlow
* Esp32
* C/C++
* MQTT

### Dados utilizados
Variável | Significado
--------- | ------
PDRi | Taxa de envio de pacotes 
RSSI | Potência do Sinal
Erro | Erro do controle


### Pré Processamento dos dados
O pré processamento dos dados estão detalhados [aqui](https://github.com/Moises788/LAMP-IC-UFRN/blob/main/Pr%C3%A9processamento/Data_analyze.ipynb)

### Treinamento da rede neural

O treinamento da rede neural, se baseia no algoritmo LSTM.

O seu desenvolvimento se encontra [aqui](https://github.com/Moises788/LAMP-IC-UFRN/blob/main/Desenvolvimento%20do%20Modelo/Modelo%20Erro/Model_Developer.ipynb)

### Resultados
Até o momento, o melhor modelo se encontra com essas características:

 
--------- | ------
Número de Neurônios | 20x20
Epocas de Treinamento | 2810
MAE |0.66478467
MSE |1.0146297
RMSE |0.60217106


