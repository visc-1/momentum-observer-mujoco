import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- 1. FUNZIONE DI PLOT AUSILIARIA (per non ripetere il codice) ---
def plot_subplot(ax, dataframe, title, ylabel, y_cols_prefix, legend_prefix):
    """
    Funzione per plottare un singolo subplot.
    - ax: L'asse matplotlib su cui disegnare.
    - dataframe: Il DataFrame pandas con i dati.
    - title: Il titolo del subplot.
    - ylabel: L'etichetta dell'asse y.
    - y_cols_prefix: Il prefisso delle colonne da plottare (es. 'tau_ext_').
    """
    # Trova tutte le colonne che iniziano con il prefisso
    columns_to_plot = [col for col in dataframe.columns if col.strip().startswith(y_cols_prefix)]
    
    # Se c'è una sola colonna da plottare (come per l'osservatore di energia)
    if len(columns_to_plot) == 1:
        ax.plot(dataframe["Time"], dataframe[columns_to_plot[0]], label=legend_prefix)
    else: # Altrimenti, plotta tutte le componenti
        for col in columns_to_plot:
            # Estrai il numero del giunto dal nome della colonna per la legenda
            joint_index = col.strip().split('_')[-1]
            ax.plot(dataframe["Time"], dataframe[col], label=f'${legend_prefix}_{{{joint_index}}}(t)$')
            
    ax.set_title(title, fontsize=18)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', fontsize='18', framealpha=1.0)
    # Imposta i limiti dell'asse x come nell'immagine di esempio
    ax.set_xlim(1.5, 7)

# --- 2. CARICAMENTO DATI ---
# Assumiamo di avere file CSV separati per ogni set di dati
try:
    wrench = pd.read_csv("wrench.csv")
    reconstructed_wrench = pd.read_csv("reconstructed_wrench.csv")
    tau_ext = pd.read_csv("nominal_r.csv")
    r = pd.read_csv("r.csv") 
    collision_link = pd.read_csv("collision_link.csv")

except FileNotFoundError as e:
    print(f"Errore: file non trovato. Assicurati che tutti i file CSV siano presenti. Dettagli: {e}")
    exit()

# --- 3. CREAZIONE DELLA FIGURA E DEI SUBPLOT ---
# `subplots(5, 1)` crea 5 righe, 1 colonna di grafici.
# `figsize` controlla la dimensione totale della figura.
# `sharex=True` fa sì che tutti i subplot condividano lo stesso asse x (molto utile!).
fig, axes = plt.subplots(5, 1, figsize=(14, 20), sharex=True)

# --- 4. PLOT DI OGNI SUBPLOT ---

# Subplot 1:
plot_subplot(axes[0], wrench, r"Forza Esterna $\mathcal{F}_{\text{ext}}$", "", "f_", "f")

# Subplot 2:
plot_subplot(axes[1], reconstructed_wrench, r"Forza Ricostruita dall'Osservatore $\hat{\mathcal{F}}_{\text{ext}}$", "", "f_", r"\hat{f}")

# Subplot 3:
plot_subplot(axes[2], tau_ext, r"Coppie d'Interazione", "Nm", "nr_", r"\tau")

# Subplot 4:
plot_subplot(axes[3], r, "Residuo", "Nm", "r_", "r")

# Subplot 5:
plot_subplot(axes[4], collision_link, "Link in Collisione", " ", "l", "l")


# --- 5. RIFINITURE FINALI ---
# Aggiungi l'etichetta all'asse x solo all'ultimo subplot
axes[-1].set_xlabel("Tempo (s)", fontsize=18)

# Ottimizza lo spazio tra i subplot
plt.tight_layout(pad=2.0)

# Salva la figura in alta qualità
plt.savefig("grafici.png", dpi=300)

# Mostra il grafico
plt.show()