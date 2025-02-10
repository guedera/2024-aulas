import numpy as np

def f(B):
    return 4 * np.sin(5*B / 2) + 5 * np.sin(B)

def bisseccao(func, a, b):
    tolerancia=0.000001
    Max=100

    # if func(a) * func(b) > 0:
    #     print("Não há mudança de sinal no intervalo dado.")
    #     return None
    
    for i in range(Max):
        p_medio = (a + b) / 2
        if abs(b - a) < tolerancia or abs(func(p_medio)) < tolerancia:
            return p_medio
        
        if func(a) * func(p_medio) < 0:
            b = p_medio
        else:
            a = p_medio
        
        i += 1
    return (a + b) / 2

B_min = -4 * np.pi
B_max = 0

n_intervals = 1000  
intervals = np.linspace(B_min, B_max, n_intervals)

raizes = []

for i in range(len(intervals) - 1):
    B1 = intervals[i]
    B2 = intervals[i + 1]
    if f(B1) * f(B2) < 0:
        raiz = bisseccao(f, B1, B2)
        raizes.append(raiz)

print("Raízes encontradas no intervalo [-4π, 0]:")
for raiz in raizes:
    print(f"B = {raiz}")

