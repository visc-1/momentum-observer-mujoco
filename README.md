# Momentum Observer C++: Implementazione per Manipolatori e Umanoidi

Questo repository ospita l'implementazione C++ dell'**Osservatore basato sul Momento Generalizzato**, validata tramite il simulatore fisico **MuJoCo** e la libreria **Pinocchio**. Il codice è parte integrante della Tesi di Laurea Triennale in Ingegneria Informatica e Automatica: **"Stima delle Forze di Collisione: Un'Analisi Comparativa di Metodi Basati su Energia, Velocità e Momento"**.

**Autore:** Francesco Viscione  
**Relatore:** Prof. Giuseppe Oriolo  
**Correlatore:** Dott. Nicola Scianca
**Università:** Sapienz Università di Roma - Dipartimento di Ingegneria Informatica, Automatica e Gestionale (DIAG)

## 📄 Documentazione
*   **[Scarica la Tesi Completa (PDF)](./tesi_viscione_2084759.pdf)**

## 📦 Contenuto
Il repository include:
*   `MomentumObserver`: Una classe C++ generica per la stima delle coppie d'interazione esterne e la localizzazione del contatto.
*   Simulazione su **KUKA LBR iiwa 14**: Validazione delle capacità di *Rilevamento, Isolamento e Identificazione*.

## ⚙️ Dipendenze
Per compilare ed eseguire il progetto sono necessarie le seguenti librerie:
*   [MuJoCo](https://mujoco.org/) (Motore fisico)
*   [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (Dinamica rigida e algoritmi spaziali)
*   [Eigen3](https://eigen.tuxfamily.org/) (Algebra lineare)
*   [GLFW3](https://www.glfw.org/) (Gestione finestre, contesto OpenGL e input)
*   CMake (Sistema di build)

## 🔨 Compilazione
```bash
cd build
cmake ..
make
```
## ▶️ Esecuzione
```bash
./main_iiwa
```
## 📈 Risultati Principali
Il codice dimostra come l'osservatore basato sul momento permetta di Rilevare, Isolare e Identificare la collisione.

---
&copy; 2025 Francesco Viscione.
