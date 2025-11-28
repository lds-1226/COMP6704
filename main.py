# main.py
import time
from benders import BendersDecomposition
from utils import generate_kmg_instance, save_results

def main():
    print("Generating Kunming Changshui (KMG) test instance...")
    flights, gates, G_taxi = generate_kmg_instance(n_flights=200)  # 可改 100~512
    
    print(f"Instance: {len(flights)} flights, {len(gates)} gates")
    
    # ---------- 经典 Benders ----------
    start = time.time()
    benders = BendersDecomposition(flights, gates, G_taxi, accelerated=False)
    obj_classic, time_classic, iters_classic = benders.solve()
    classic_time = time.time() - start
    
    # ---------- 加速版 Benders (Magnanti-Wong + per-flight cuts) ----------
    start = time.time()
    benders_acc = BendersDecomposition(flights, gates, G_taxi, accelerated=True)
    obj_acc, time_acc, iters_acc = benders_acc.solve()
    acc_time = time.time() - start
    
    # 保存结果
    save_results(obj_classic, obj_acc, classic_time, acc_time, iters_classic, iters_acc, len(flights))
    print("All done! Results saved in ./results/")

if __name__ == "__main__":
    main()
