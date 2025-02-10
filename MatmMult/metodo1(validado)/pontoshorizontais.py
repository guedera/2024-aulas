import numpy as np

# Definindo as funções da curva
def gamma_x(t):
    return 5 * np.cos(t) - 4 * np.cos(5 * t / 2)

def gamma_y(t):
    return 5 * np.sin(t) - 4 * np.sin(5 * t / 2)

def gamma_linha_y(t):
    return 5 * np.cos(t) - 10 * np.cos(5 * t / 2)

#Filtra lista para remover valores encontrados que são muito parecidos de t1[x] e t1[x+1] e etc

def filtrar_lista(lista, limite=0.1):
    nova_lista = [lista[0]]  
    for i in range(1, len(lista)):
        if abs(lista[i] - lista[i - 1]) >= limite:
            nova_lista.append(lista[i])
    return nova_lista

#Intervalos de t para achar os pontos de intersec
t_values = np.linspace(0, 4 * np.pi, 100000)

#listasvazias
intersectionst1 = []

for t1 in t_values:
        if (gamma_linha_y(t1)<0.001) and (gamma_linha_y(t1)>-0.001):
                if t1 not in intersectionst1:
                    intersectionst1.append(t1)

#filtra com a funcao para tirar os valores parecidos
intersectionst1 = filtrar_lista(intersectionst1)

#imprime os valores de t's
for i in (range(len(intersectionst1))):
    print("tangencia horizontal em t = "+ str(intersectionst1[i]))

pontos = []

#imprime quantos t's existem
print("\n Achei: " + str(len(intersectionst1)) + " valores de t's\n")

for i in (range(len(intersectionst1))):
    j = str("(" + str(round(gamma_x(intersectionst1[i]),2))+","+str(round(gamma_y(intersectionst1[i]),2))+")")
    pontos.append(j)

for i in range(len(pontos)):
    print("H" + str(i+1) + " = " + str(pontos[i]))