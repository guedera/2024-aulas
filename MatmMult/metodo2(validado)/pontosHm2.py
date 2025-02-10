import numpy as np

def encontrar_zeros(f, inicio=0, fim=4*np.pi, passo=0.0001, tolerancia=1e-3):
    zeros = []
    ultimo_zero = None
    
    for t in np.arange(inicio, fim, passo):
        valor_f = f(t)
        if abs(valor_f) < tolerancia:
            # Verifica se já não encontrou um zero próximo
            if ultimo_zero is None or abs(t - ultimo_zero) > passo:
                zeros.append(round(t, 3))  # Arredonda para 3 casas decimais
                ultimo_zero = t
    
    # Exibe os resultados formatados com 3 casas decimais
    print(f"Valores de t onde f(t) = 0 (aproximadamente): {zeros}")
    return zeros

f = lambda t: (5 * np.cos(t) - 10 * np.cos(5 * t / 2))

encontrar_zeros(f)

