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
    private final long RUNTIME = 600; // seconds; 10 minutes
    private static int BIN_ITER = 7; 
    private final long LAG_ITER = 2000; 

    private SliceChallengeSolver sliceSolver;
    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    

    public static double f(int x) {
        int xMin = 0;
        int xMax = BIN_ITER-1;

        double logStart = Math.log(BIN_ITER - xMin + 1); // ln(11)
        double logEnd = Math.log(BIN_ITER - xMax + 1);   // ln(2)
        double logX = Math.log(BIN_ITER - x + 1);        // ln(11 - x)

        double normalized = (logX - logEnd) / (logStart - logEnd);
        // System.out.println("oi "+(20*(1.0 - normalized))+"\n");
        return normalized;
    }

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;

        this.sliceSolver = new SliceChallengeSolver(orders, aisles, nItems, waveSizeLB, waveSizeUB);
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        long totalItens = 0;

        for(int o = 0 ; o < orders.size() ; o++){
            for(var entry: orders.get(o).entrySet()){
                totalItens += entry.getValue();
            }
        }

        if(totalItens < 2 * orders.size()){
            return sliceSolver.solve(stopWatch);
        }

       // Build item-oriented maps
        Map<Integer,Integer>[] OrderItems = new HashMap[nItems];
        Map<Integer,Integer>[] AisleItems = new HashMap[nItems];
        for(int i=0;i<nItems;i++){
            OrderItems[i]=new HashMap<>();
            AisleItems[i]=new HashMap<>();
        }
        for(int o=0;o<orders.size();o++){
            for(Map.Entry<Integer,Integer> e: orders.get(o).entrySet()){
                OrderItems[e.getKey()].put(o,e.getValue());
            }
        }
        for(int a=0;a<aisles.size();a++){
            for(Map.Entry<Integer,Integer> e: aisles.get(a).entrySet()){
                AisleItems[e.getKey()].put(a,e.getValue());
            }
        }
        boolean mark = false;
        double bestOverallPrimal = Double.NEGATIVE_INFINITY;
        double dual = Double.NEGATIVE_INFINITY;
        double alpha = 1e-8;
        double epsilon = 1e-6;
        Set<Integer> bestOrders = new HashSet<>();
        Set<Integer> bestAisles = new HashSet<>();
        IloObjective objective = null;
        double gap = 0;
        IloCplex.Status stat = null;
        try {
            IloCplex cplex = new IloCplex();
            cplex.setOut(null);
            cplex.setWarning(null);
            cplex.setParam(IloCplex.IntParam.MIP.Display, 0);
            // cplex.setParam(IloCplex.IntParam.MIP.Interval, 0);
            // cplex.setParam(IloCplex.IntParam.MIP.Limits.TreeMemory, 4096);
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
            IloNumVar [] aisvar = cplex.numVarArray(aisles.size(),0,1, IloNumVarType.Bool);

            
            IloLinearNumExpr sumorders = cplex.linearNumExpr();
            for (int i = 0; i < orders.size(); i++) {
                for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                    sumorders.addTerm(entry.getValue(),ordvar[i]);
                }
            }
            IloLinearNumExpr sumaisles = cplex.linearNumExpr();
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
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.1);
            // cplex.setParam(IloCplex.DoubleParam.WorkMem, 14000); // limitar uso de memoria pra 14 GB
            // // Descobrir o número de núcleos da CPU
            // int totalCores = Runtime.getRuntime().availableProcessors();
            // int maxThreads = (totalCores*8+9)/10; // 80% dos núcleos
            // // Limitar o número de threads do CPLEX
            // cplex.setParam(IloCplex.IntParam.Threads, maxThreads);
            IloLinearNumExpr objetivo = cplex.linearNumExpr();
            for(int c = 0; c < BIN_ITER; c++)
            {

                if(getRemainingTime(stopWatch) <=20)
                    break;
                // cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, f(c));
                // if(c == 4)
                // {
                // }
                // if(c != 0){
                    // System.out.println("ALO7 "+cplex.getObjValue());
                // }
                // double mid = l;
                // System.out.println("\n -------Execução " + c+" alpha "+alpha+" Tempo Disponivel "+(getRemainingTime(stopWatch)-20)/(BIN_ITER-c)+" Real time "+getRemainingTime(stopWatch)+" ------\n");
                // System.out.println("\n -------Execução " + c+" alpha "+alpha+" Tempo Disponivel "+getRemainingTime(stopWatch)+" gap "+ f(c)+" ------\n");
                if(c == 0 || Math.abs(cplex.getObjValue()) > epsilon)
                {
                    if(c != 0){
                        // cplex.addGe(objetivo, cplex.getObjValue());
                        cplex.remove(objective);
                        cplex.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Dual);
                    }
                    objetivo = cplex.linearNumExpr();
                    for (int i = 0; i < orders.size(); i++) {
                        for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                            objetivo.addTerm(entry.getValue(),ordvar[i]);
                        }
                    }
                    
                    for (int i = 0; i < aisles.size(); i++) {
                        objetivo.addTerm(-alpha,aisvar[i]);
                    }

                    
                    objective = cplex.addMaximize(objetivo);
                }

                // TIRANDO TLE PRA BRUTAR

                cplex.setParam(IloCplex.DoubleParam.TiLim, (getRemainingTime(stopWatch)-20)/(BIN_ITER-c));
                if (cplex.solve()) {
                    // System.out.println("Valor ótimo = " + cplex.getObjValue());
                    double g=0;
                    for(int a=0;a<aisles.size();a++){
                        if(cplex.getValue(aisvar[a])>0.5){ g++; }
                    }
                    alpha = alpha + cplex.getObjValue()/g;
                    if(cplex.getStatus() == IloCplex.Status.Optimal && Math.abs(cplex.getObjValue()) < epsilon){
                        mark = true;
                        break;
                    }
                    // if(Math.abs(cplex.getObjValue()) > epsilon)
                    // System.out.println("ALO1");
                } else {
                    cplex.end();
                    System.out.println("Não foi encontrada solução viável.");
                    return null;
                } 
            }
            


           
            bestOrders.clear();
            bestAisles.clear();
            for (int i = 0; i < orders.size(); i++) {
                if(cplex.getValue(ordvar[i]) > 0.5)
                    bestOrders.add(i);
            }
            for (int i = 0; i < aisles.size(); i++) {
                if(cplex.getValue(aisvar[i]) > + 0.5)
                    bestAisles.add(i);
            }
            bestOverallPrimal=alpha;
            gap = cplex.getMIPRelativeGap();
            stat = cplex.getStatus();
            cplex.end();
            // try (BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt", true))) {
            //         writer.write("-------Resultado: ratio "+bestOverallPrimal+ " GAP "+gap + " Otima? "+stat+ " Tempo gasto: " + stopWatch.getTime(TimeUnit.SECONDS) + "s" +" ------\n");   
            // } catch (IOException e) {
            //     e.printStackTrace();
            // }
            // if(mark)
            // {
            //     // try (BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt", true))){

            //     //     writer.write("parei antes patrão\n");
            //     // }catch (IOException e) {
            //     //     e.printStackTrace();
            //     // }
            //     return new ChallengeSolution(bestOrders,bestAisles);
            // }
            
        } catch (IloException e) {
            e.printStackTrace();
        }

        

        
        // try {
        //     for(int outer=0; outer<BIN_ITER; outer++){
        //         System.out.println("\n -------Execução " + outer+" alpha "+alpha+" Real time "+getRemainingTime(stopWatch)+" ------\n");
        //         double[] lambda = new double[nItems];
        //         double stepSize = 2.0;
        //         double bestPrimal = Double.NEGATIVE_INFINITY;
        //         double bestDual = Double.NEGATIVE_INFINITY;
        //         for (int i = 0; i < nItems; i++) {
        //             double lo = 0, hi = 0;
        //             for (Map.Entry<Integer, Integer> e : OrderItems[i].entrySet()) {
        //                 if (bestOrders.contains(e.getKey())) {
        //                     lo += e.getValue();
        //                 }
        //             }
        //             for (Map.Entry<Integer, Integer> e : AisleItems[i].entrySet()) {
        //                 if (bestAisles.contains(e.getKey())) {
        //                     hi += e.getValue();
        //                 }
        //             }
        //             lambda[i] = lo-hi;
        //         }
        //         for(int inner=0; inner<LAG_ITER; inner++){
        //             if(getRemainingTime(stopWatch)<=5) break;
        //             IloCplex cplex = new IloCplex();
        //             cplex.setOut(null);
        //             cplex.setWarning(null);
        //             cplex.setParam(IloCplex.IntParam.MIP.Display, 0);
        //             // cplex.setParam(IloCplex.IntParam.MIP.Interval, 0);
        //             // cplex.setParam(IloCplex.IntParam.MIP.Limits.TreeMemory, 0);
        //             // Variables
        //             IloNumVar[] ordvar = new IloNumVar[orders.size()];
        //             for (int i = 0; i < orders.size(); i++) {
        //                 ordvar[i] = cplex.numVar(0,1, IloNumVarType.Bool,"ord_" + i);
        //             }
        //             IloNumVar[] aisvar = new IloNumVar[aisles.size()];
        //             for (int a = 0; a < aisles.size(); a++) {
        //                 aisvar[a] = cplex.numVar(0,1, IloNumVarType.Bool,"ais_" + a);
        //             }
        //             // Começar com esses
        //             // if(!bestOrders.isEmpty() || !bestAisles.isEmpty()) {
        //             int nOrd = orders.size();
        //             int nAis = aisles.size();
        //             IloNumVar[] allVars = new IloNumVar[nOrd + nAis];
        //             double[] allVals = new double[nOrd + nAis];
        //             for (int i = 0; i < nOrd; i++) {
        //                 allVars[i] = ordvar[i];
        //                 allVals[i] = bestOrders.contains(i) ? 1.0 : 0.0;
        //             }
        //             for (int j = 0; j < nAis; j++) {
        //                 allVars[nOrd + j] = aisvar[j];
        //                 allVals[nOrd + j] = bestAisles.contains(j) ? 1.0 : 0.0;
        //             }
        //             // Use NoCheck to force acceptance of the start
        //             // }
        //             // Expressions
        //             IloLinearNumExpr sumorders = cplex.linearNumExpr();
        //             IloLinearNumExpr sumaisles = cplex.linearNumExpr();
        //             IloLinearNumExpr lagr = cplex.linearNumExpr();
        //             // Build
        //             for(int o=0;o<orders.size();o++){
        //                 double coeff=0;
        //                 for(Map.Entry<Integer,Integer> e: orders.get(o).entrySet()){
        //                     int i = e.getKey();
        //                     int u = e.getValue();
        //                     coeff += u * (1 - lambda[i]);
        //                     sumorders.addTerm(u, ordvar[o]);
        //                 }
        //                 lagr.addTerm(coeff, ordvar[o]);
        //             }
        //             for(int a=0;a<aisles.size();a++){
        //                 double coeff=0;
        //                 for(Map.Entry<Integer,Integer> e: aisles.get(a).entrySet()){
        //                     int i = e.getKey();
        //                     int u = e.getValue();
        //                     coeff += lambda[i] * u;
        //                 }
        //                 sumaisles.addTerm(1, aisvar[a]);
        //                 lagr.addTerm(coeff - alpha, aisvar[a]);
        //             }
        //             cplex.addMaximize(lagr);
        //             // Constraints
        //             cplex.addGe(sumaisles,1);
        //             cplex.addGe(sumorders, waveSizeLB);
        //             cplex.addLe(sumorders, waveSizeUB);
                    
        //             // Time limit per iteration
        //             cplex.setParam(IloCplex.DoubleParam.TiLim, (getRemainingTime(stopWatch)-5.0)/(BIN_ITER*LAG_ITER-outer*LAG_ITER - inner));
        //             cplex.addMIPStart(allVars, allVals, IloCplex.MIPStartEffort.NoCheck);
                    
        //             if(!cplex.solve()){
        //                 cplex.end(); break;
        //             }
        //             // Evaluate solution
        //             double f=0,g=0;
        //             Set<Integer> usedO=new HashSet<>(), usedA=new HashSet<>();
        //             for(int o=0;o<orders.size();o++){
        //                 if(cplex.getValue(ordvar[o])>0.5){
        //                     usedO.add(o);
        //                     for(Map.Entry<Integer,Integer> e: orders.get(o).entrySet()) f+=e.getValue();
        //                 }
        //             }
        //             for(int a=0;a<aisles.size();a++){
        //                 if(cplex.getValue(aisvar[a])>0.5){ usedA.add(a); g++; }
        //             }
        //             double phi = f - alpha*g;
        //             bestDual = Math.min(bestDual, phi); 
        //             boolean feasible=true;
        //             for(int i=0;i<nItems;i++){
        //                 double lo=0, hi=0;
        //                 for(Map.Entry<Integer,Integer> e: OrderItems[i].entrySet()) lo+=e.getValue()*cplex.getValue(ordvar[e.getKey()]);
        //                 for(Map.Entry<Integer,Integer> e: AisleItems[i].entrySet()) hi+=e.getValue()*cplex.getValue(aisvar[e.getKey()]);
        //                 if(lo>hi+epsilon){
        //                     // System.out.printf("Item %d: LHS=%.2f, RHS=%.2f, slack=%.2f%n", i, lo, hi, hi-lo);
        //                     feasible=false;
        //                 } 
        //             }
        //             if(feasible && g >= 1){
        //                 double ratio=f/g;
        //                 if(ratio>bestPrimal){
        //                     bestPrimal=ratio;
        //                     bestOrders=new HashSet<>(usedO);
        //                     bestAisles=new HashSet<>(usedA);
        //                     gap = cplex.getMIPRelativeGap();
        //                     stat = cplex.getStatus();
        //                 }
        //             }
        //             // Subgradient update
        //             double[] subg=new double[nItems];
        //             double normSq = 0;
        //             for(int i=0;i<nItems;i++){
        //                 double lo=0, hi=0;
        //                 for(Map.Entry<Integer,Integer> e: OrderItems[i].entrySet()) lo+=e.getValue()*cplex.getValue(ordvar[e.getKey()]);
        //                 for(Map.Entry<Integer,Integer> e: AisleItems[i].entrySet()) hi+=e.getValue()*cplex.getValue(aisvar[e.getKey()]);
        //                 subg[i] = hi - lo;
        //                 normSq += subg[i] * subg[i];
        //             }
        //             // Adaptive step size: Polyak's rule
        //             double tk;
        //             if(normSq>1e-6) tk=Math.max(1e-6,(bestDual-phi)/normSq);
        //             else tk=stepSize/(inner+1);
        //             for(int i=0;i<nItems;i++) lambda[i]=Math.min(0, lambda[i]-tk*subg[i]);
        //             // System.out.printf("Outer %d Inner %d: f=%.2f,g=%.2f,phi=%.2f,feasible=%b\n",outer,inner,f,g,phi,feasible);
        //             // System.out.println(" Orders="+usedO+"\n Aisles="+usedA+"\n");
        //             cplex.end();
        //         }
        //         if(bestPrimal>bestOverallPrimal){
        //             System.out.println("ALELUIA\n"); 
        //             bestOverallPrimal=bestPrimal; 
        //             alpha=bestPrimal; 
        //         }
        //     }
        // } catch(IloException e){ 
        //     e.printStackTrace(System.out); 
        // }
        // System.out.println(" Orders="+bestOrders+"\n Aisles="+bestAisles+"\n");

        // try (BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt", true))) {
        //         writer.write("-------Resultado: ratio "+bestOverallPrimal+ " GAP "+gap + " Otima? "+stat+ " Tempo gasto: " + stopWatch.getTime(TimeUnit.SECONDS) + "s" +" ------\n\n");   
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }
        return new ChallengeSolution(bestOrders,bestAisles);
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
