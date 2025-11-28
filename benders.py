# benders.py
import gurobipy as gp
from gurobipy import GRB
import numpy as np

class BendersDecomposition:
    def __init__(self, flights, gates, G_taxi, accelerated=True, tol=1e-4):
        self.F = flights
        self.G = gates
        self.graph = G_taxi
        self.acc = accelerated
        self.tol = tol
        
    def solve(self, max_iters=200):
        master = self.build_master()
        lb_history, ub_history = [], []
        best_ub = GRB.INFINITY
        
        for it in range(max_iters):
            master.optimize()
            if master.status != GRB.OPTIMAL:
                print("Master infeasible!"); break
                
            x_sol = {f: {g: master.getVarByName(f"x[{f.id},{g.id}]").X 
                        for g in self.G} for f in self.F}
            lb = master.ObjVal
            lb_history.append(lb)
            
            # 解子问题（每航班独立）
            ub, cuts = self.solve_subproblems(x_sol)
            ub_history.append(ub)
            best_ub = min(best_ub, ub)
            
            print(f"Iter {it+1:3d} | LB: {lb:8.2f} | UB: {ub:8.2f} | Gap: {(ub-lb)/ub*100:5.2f}%")
            
            if (ub - lb) / max(1, abs(ub)) <= self.tol:
                print("Optimal solution found!")
                break
                
            # 添加 Benders cuts
            for cut_type, expr, rhs in cuts:
                if cut_type == "opt":
                    master.addConstr(master.getVarByName("eta") >= expr, name=f"opt_cut_{it}")
                else:  # feasibility cut
                    master.addConstr(expr <= rhs, name=f"fea_cut_{it}")
                    
            if self.acc and it % 5 == 0:  # 每5轮生成一次 Pareto-optimal cut
                self.add_magnanti_wong_cut(master, x_sol)
                
        return best_ub, sum(ub_history[-10:])/10, len(lb_history)  # 返回最终 UB
    
    def build_master(self):
        m = gp.Model("GateMaster")
        m.Params.OutputFlag = 0
        
        x = m.addVars(self.F, self.G, vtype=GRB.BINARY, name="x")
        eta = m.addVar(lb=-GRB.INFINITY, name="eta")
        
        # 每个航班一个登机口
        for f in self.F:
            m.addConstr(gp.quicksum(x[f,g] for g in self.G) == 1, name=f"one_gate_{f.id}")
            
        # 登机口不相容性 & 缓冲时间（简化版）
        for g in self.G:
            flights_g = [f for f in self.F if g.compatible(f)]
            for i in range(len(flights_g)):
                for j in range(i):
                    f1, f2 = flights_g[j], flights_g[i]
                    if f1.departure + 15 > f2.arrival:  # 15min buffer
                        m.addConstr(x[f1,g] + x[f2,g] <= 1)
        
        m.setObjective(eta, GRB.MINIMIZE)
        return m
        
    def solve_subproblems(self, x_sol):
        total_taxi = 0.0
        cuts = []
        
        for f in self.F:
            g_assigned = next(g for g in self.G if x_sol[f][g] > 0.5)
            # 构建单航班最短无冲突路径子问题（时间离散化 15s）
            sp = self.build_taxi_subproblem(f, g_assigned)
            sp.optimize()
            
            if sp.status == GRB.INFEASIBLE:
                # 生成可行性割
                sp.computeIIS()
                fcut = sp.FarkasDual
                expr = sum(fcut[f"x[{f.id},{g.id}]"] * x_sol[f][g] for g in self.G)
                cuts.append(("fea", expr, 0))
            else:
                total_taxi += sp.ObjVal
                pi = {c.ConstrName: c.Pi for c in sp.getConstrs()}
                # 经典 optimality cut
                expr = sp.ObjVal + sum(pi[f"x[{f.id},{g.id}]"] * 
                                     (master.getVarByName(f"x[{f.id},{g.id}]") - x_sol[f][g]) 
                                     for g in self.G)
                cuts.append(("opt", expr, 0))
                
        return total_taxi, cuts
        
    def build_taxi_subproblem(self, flight, gate):
        # 极简版：时间窗内最短路径 + 冲突避免（完整版可扩展为多航班时间-空间网络）
        m = gp.Model(f"TaxiSP_{flight.id}")
        m.Params.OutputFlag = 0
        # 这里用 A* + 速度调整的启发式近似真实最短时间（作业足够）
        return m  # 占位，实际你可以用 networkx + gurobi 建时间扩展图
