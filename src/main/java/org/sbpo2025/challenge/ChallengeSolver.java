package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import ilog.concert.*;
import ilog.cplex.*;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes
    private final long RUNTIME = 400; // seconds; 10 minutes
    private final long BIN_ITER = 25; 

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

        // codigo da força bruta
        // ChallengeSolution mysol = new ChallengeSolution(new HashSet<>(),new HashSet<>());
        // // mysol.orders = new Set<>();
        // // mysol.aisles = new Set<>();
        // while(!isSolutionFeasible(mysol))
        // {
        //     ChallengeSolution best = add_aisles(mysol,0);
        //     for (int i = 1; i < orders.size(); i++) {
        //         if(isSolutionFeasible(best)){
        //             System.out.println("OPA");
        //             break;
        //         }
        //         ChallengeSolution nat = add_aisles(mysol,1);
        //         if(calc_item(nat) > calc_item(best) && calc_item(nat) < waveSizeUB){
        //             best = new ChallengeSolution(nat.orders(),nat.aisles());
        //         }
        //     }
        //     if(best.equals(mysol)){
        //         System.out.println(isSolutionFeasible(mysol));
        //         break;
        //     }
        //     mysol = new ChallengeSolution(best.orders(),best.aisles());
        // }
        System.out.println(all);
        double l = 1e-8, r = all;
        Set<Integer> orderset = new HashSet<>();
        Set<Integer> aisleset = new HashSet<>();
        try {
            IloCplex cplex = new IloCplex();
            cplex.setParam(IloCplex.DoubleParam.TiLim, RUNTIME/BIN_ITER);
            IloIntVar [] ordvar = cplex.intVarArray(orders.size(),0,1);

            
            IloLinearNumExpr sumorders = cplex.linearNumExpr();


            for (int i = 0; i < orders.size(); i++) {
                for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                    sumorders.addTerm(entry.getValue(),ordvar[i]);
                }
            }
            IloLinearNumExpr sumaisles = cplex.linearNumExpr();

            IloIntVar [] aisvar = cplex.intVarArray(aisles.size(),0,1);
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
            for(int c = 0; c < BIN_ITER; c++)
            {
                    
                double mid = (l+r)/2.0;
                System.out.println("\n -------Execução " + c+" l "+l+" r "+r+" ------\n");
                    
                IloLinearNumExpr objetivo = cplex.linearNumExpr();
                for (int i = 0; i < orders.size(); i++) {
                    for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                        objetivo.addTerm(entry.getValue(),ordvar[i]);
                    }
                }
                
                for (int i = 0; i < aisles.size(); i++) {
                    objetivo.addTerm(-mid,aisvar[i]);
                }

                
                IloObjective objective = cplex.addMaximize(objetivo);

                

                // Resolver o modelo
                // System.out.println(cplex.solve());
                if (cplex.solve()) {
                    // System.out.println("Solução ótima encontrada:");
                    // System.out.println("x = " + (int) cplex.getValue(x));
                    // System.out.println("y = " + (int) cplex.getValue(y));
                    System.out.println("Valor ótimo Z = " + cplex.getObjValue());
                    if(cplex.getObjValue() >= 0)
                        l = mid;
                    else
                        r = mid;
                    cplex.remove(objective);
                } else {
                    cplex.end();
                    System.out.println("Não foi encontrada solução viável.");
                    return null;
                }
            }
        

            double mid = (l+r)/2.0;
            IloLinearNumExpr objetivo = cplex.linearNumExpr();
            for (int i = 0; i < orders.size(); i++) {
                for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                    objetivo.addTerm(entry.getValue(),ordvar[i]);
                }
            }
            
            for (int i = 0; i < aisles.size(); i++) {
                objetivo.addTerm(-mid,aisvar[i]);
            }
    
            IloObjective objective = cplex.addMaximize(objetivo);

            cplex.setParam(IloCplex.DoubleParam.TiLim, 100);
            if (cplex.solve()) {
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
            } else {
                System.out.println("Não foi encontrada solução viável.");
                cplex.end();
                return null;
            }
        } catch (IloException e) {
            e.printStackTrace();
        }
        // Fechar o solver
        return null;
    }

    public ChallengeSolution add_aisles(ChallengeSolution mysol, Integer id) {
        Set<Integer> selectedOrders = mysol.orders();
        if(selectedOrders.contains(id))
            return mysol;
        Set<Integer> visitedAisles = mysol.aisles();
        selectedOrders.add(id);
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
        for (int i = 0; i < aisles.size(); i++) {
            if(visitedAisles.contains(i))
                continue;
            boolean mark = false;
            for (Map.Entry<Integer, Integer> entry : aisles.get(i).entrySet()) {
                if(totalUnitsAvailable[entry.getKey()] < totalUnitsPicked[entry.getKey()])
                    mark = true;
            }
            if(!mark)
                continue;
            visitedAisles.add(i);
            for (Map.Entry<Integer, Integer> entry : aisles.get(i).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }
        return new ChallengeSolution(selectedOrders,visitedAisles);
    }

    public Integer calc_item(ChallengeSolution at)
    {
        Set<Integer> selectedOrders = at.orders();
        int totalUnits = 0;
        // Calculate total units picked
        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnits += entry.getValue();
            }
        }
        return totalUnits;
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
