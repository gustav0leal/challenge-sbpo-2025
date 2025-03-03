package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import ilog.concert.*;
import ilog.cplex.*;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes
    private final long RUNTIME = 550; // seconds; 10 minutes
    private final long BIN_ITER = 5; 

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        Map<Integer, Integer>[] OrderItems = new HashMap[nItems];
        Map<Integer, Integer>[] AisleItems = new HashMap[nItems];
        int all = 0;
        for (int i = 0; i < nItems; i++) {
            OrderItems[i] = new HashMap<>();
            AisleItems[i] = new HashMap<>();
        }
        for (int i = 0; i < orders.size(); i++) {
            for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                all += entry.getValue();
                // System.out.println(entry.getKey()+ " " + nItems);
                OrderItems[entry.getKey()].put(i,entry.getValue());
            }
        }

        for (int i = 0; i < aisles.size(); i++) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(i).entrySet()) {
                AisleItems[entry.getKey()].put(i,entry.getValue());
            }
        }

        
        System.out.println(all);
        double l = 1e-8;
        double epsilon = 1e-6;
        boolean mark = false;
        Set<Integer> orderset = new HashSet<>();
        Set<Integer> aisleset = new HashSet<>();
        IloObjective objective = null;
        try {
            IloCplex cplex = new IloCplex();
            // cplex.setParam(IloCplex.DoubleParam.TiLim, RUNTIME/BIN_ITER);
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.HeuristicFreq, 20);
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.RINSHeur, 10);
            // cplex.setParam(IloCplex.BooleanParam.MIP.Strategy.LBHeur, true);
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.NodeSelect, 1);
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.VariableSelect, 3);
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.Dive, 3);
            // cplex.setParam(IloCplex.IntParam.MIP.Limits.StrongIt, 10); 
            // cplex.setParam(IloCplex.IntParam.MIP.Strategy.Branch, 3);
            IloNumVar [] ordvar = cplex.numVarArray(orders.size(),0,1, IloNumVarType.Bool);

            
            IloLinearNumExpr sumorders = cplex.linearNumExpr();


            for (int i = 0; i < orders.size(); i++) {
                for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                    sumorders.addTerm(entry.getValue(),ordvar[i]);
                }
            }
            IloLinearNumExpr sumaisles = cplex.linearNumExpr();

            IloNumVar [] aisvar = cplex.numVarArray(aisles.size(),0,1, IloNumVarType.Bool);
            for (int i = 0; i < aisles.size(); i++) {
                sumaisles.addTerm(1,aisvar[i]);
            }

            cplex.addGe(sumaisles, 1);

            cplex.addGe(sumorders, waveSizeLB);

            cplex.addLe(sumorders, waveSizeUB);

            for (int i = 0; i < nItems; i++) {
                IloLinearNumExpr ordersum = cplex.linearNumExpr();
                IloLinearNumExpr aislesum = cplex.linearNumExpr();
                for (Map.Entry<Integer, Integer> entry: OrderItems[i].entrySet()) {
                    ordersum.addTerm(entry.getValue(),ordvar[entry.getKey()]);
                }
                for (Map.Entry<Integer, Integer> entry : AisleItems[i].entrySet()) {
                    aislesum.addTerm(entry.getValue(),aisvar[entry.getKey()]);
                }
                cplex.addLe(ordersum,aislesum);
            }
            double startTime = cplex.getCplexTime();
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.04);
            // cplex.setParam(IloCplex.DoubleParam.WorkMem, 14000); // limitar uso de memoria pra 14 GB
            // // Descobrir o número de núcleos da CPU
            // int totalCores = Runtime.getRuntime().availableProcessors();
            // int maxThreads = (totalCores*8+9)/10; // 80% dos núcleos
            // // Limitar o número de threads do CPLEX
            // cplex.setParam(IloCplex.IntParam.Threads, maxThreads);
            for(int c = 0; c < BIN_ITER; c++)
            {
                if(getRemainingTime(stopWatch) <= 100)
                    break;
                if(c != 0){
                    cplex.remove(objective);
                    cplex.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Dual);
                }
                cplex.setParam(IloCplex.DoubleParam.TiLim, (getRemainingTime(stopWatch)-100)/(BIN_ITER-c));
                // double mid = l;
                System.out.println("\n -------Execução " + c+" l "+l+" Tempo Disponivel "+(getRemainingTime(stopWatch)-100)/(BIN_ITER-c)+" ------\n");
                    
                IloLinearNumExpr objetivo = cplex.linearNumExpr();
                for (int i = 0; i < orders.size(); i++) {
                    for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                        objetivo.addTerm(entry.getValue(),ordvar[i]);
                    }
                }
                
                for (int i = 0; i < aisles.size(); i++) {
                    objetivo.addTerm(-l,aisvar[i]);
                }

                
                objective = cplex.addMaximize(objetivo);

                

                if (cplex.solve()) {
                    System.out.println("Valor ótimo = " + cplex.getObjValue());
                    int tot = 0;
                    for (int i = 0; i < aisles.size(); i++) {
                        if(cplex.getValue(aisvar[i]) > 0.5)
                            tot = tot+1;
                    }
                    l = l + cplex.getObjValue()/tot;
                    if(cplex.getStatus() == IloCplex.Status.Optimal && Math.abs(cplex.getObjValue()) < epsilon){
                        mark = true;
                        break;
                    }
                } else {
                    cplex.end();
                    System.out.println("Não foi encontrada solução viável.");
                    return null;
                }

                
            }
            if(!mark && getRemainingTime(stopWatch) > 100)
            {
                
                cplex.remove(objective);
                cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.0);
                
                // double mid = l;
                System.out.println("\n --- l "+l+" Tempo Disponivel "+getRemainingTime(stopWatch)+" ------\n");
                    
                IloLinearNumExpr objetivo = cplex.linearNumExpr();
                for (int i = 0; i < orders.size(); i++) {
                    for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                        objetivo.addTerm(entry.getValue(),ordvar[i]);
                    }
                }
                
                for (int i = 0; i < aisles.size(); i++) {
                    objetivo.addTerm(-l,aisvar[i]);
                }

                
                objective = cplex.addMaximize(objetivo);

                cplex.setParam(IloCplex.DoubleParam.TiLim, (getRemainingTime(stopWatch)-30));
                cplex.solve();
            }


            try (BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt", true))) {
                writer.write("\n -------Resultado: l "+l+" diff "+cplex.getObjValue()+ " GAP "+cplex.getMIPRelativeGap() + " Otima? "+cplex.getStatus() +" ------\n");
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            orderset.clear();
            aisleset.clear();
            for (int i = 0; i < orders.size(); i++) {
                if(cplex.getValue(ordvar[i]) > 0.5)
                    orderset.add(i);
            }
            for (int i = 0; i < aisles.size(); i++) {
                if(cplex.getValue(aisvar[i]) > + 0.5)
                    aisleset.add(i);
            }
            cplex.end();
            return new ChallengeSolution(orderset,aisleset);
            
        } catch (IloException e) {
            e.printStackTrace();
        }
        // Fechar o solver
        return null;
    }

    /*
     * Get the remaining time in seconds
     */
    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0);
    }

    protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return false;
        }

        int[] totalUnitsPicked = new int[nItems];
        int[] totalUnitsAvailable = new int[nItems];

        // Calculate total units picked
        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        // Calculate total units available
        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        // Check if the total units picked are within bounds
        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        // Check if the units picked do not exceed the units available
        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
                return false;
            }
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }
        int totalUnitsPicked = 0;

        // Calculate total units picked
        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream()
                    .mapToInt(Integer::intValue)
                    .sum();
        }

        // Calculate the number of visited aisles
        int numVisitedAisles = visitedAisles.size();

        // Objective function: total units picked / number of visited aisles
        return (double) totalUnitsPicked / numVisitedAisles;
    }
}
