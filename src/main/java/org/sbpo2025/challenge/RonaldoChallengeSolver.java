package org.sbpo2025.challenge;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang3.time.StopWatch;

import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPSolver;

import ilog.cplex.IloCplex;


public class RonaldoChallengeSolver extends ChallengeSolver {
    private static int BIN_ITER = 7; 

    public RonaldoChallengeSolver(List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems,
            int waveSizeLB, int waveSizeUB) {
        super(orders, aisles, nItems, waveSizeLB, waveSizeUB);
        //TODO Auto-generated constructor stub
    }
    
    @Override
    public ChallengeSolution solve(StopWatch stopWatch) {
        double infinity = java.lang.Double.POSITIVE_INFINITY;

        Loader.loadNativeLibraries();

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

        double alpha = 1e-8;
        double epsilon = 1e-6;
        Set<Integer> bestOrders = new HashSet<>();
        Set<Integer> bestAisles = new HashSet<>();
        var solver = MPSolver.createSolver("SCIP");

        var ordvar = solver.makeBoolVarArray(orders.size());
        var aisvar = solver.makeBoolVarArray(aisles.size());

        var sumorders = solver.makeConstraint(waveSizeLB, waveSizeUB, "Sum orders bounds constraint");

        for (int i = 0; i < orders.size(); i++) {
            Integer totalItemSum = 0;

            for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                totalItemSum += entry.getValue();
            }

            sumorders.setCoefficient(ordvar[i], totalItemSum);
        }

        var sumaisles = solver.makeConstraint(1.0, infinity, "At least one aisle constraint");

        for (int i = 0; i < aisles.size(); i++) {
            sumaisles.setCoefficient(aisvar[i], 1.0);
        }

        for (int i = 0; i < nItems; i++) {
            var itemConstraint = solver.makeConstraint(0.0, infinity, "Constraint for item " + i);

            for (Map.Entry<Integer, Integer> entry: OrderItems[i].entrySet()) {
                var a = entry.getValue();
                itemConstraint.setCoefficient(ordvar[entry.getKey()], -entry.getValue());
            }
            for (Map.Entry<Integer, Integer> entry : AisleItems[i].entrySet()) {
                var a = entry.getValue();
                itemConstraint.setCoefficient(aisvar[entry.getKey()], entry.getValue());
            }
        }
        
        try {
            for(int c = 0; c < BIN_ITER; c++)
            {
                if(getRemainingTime(stopWatch) <=20)
                    break;

                var objetivo = solver.objective();
                for (int i = 0; i < orders.size(); i++) {
                    Integer totalItemSum = 0;

                    for (Map.Entry<Integer, Integer> entry : orders.get(i).entrySet()) {
                        totalItemSum += entry.getValue();
                    }

                    objetivo.setCoefficient(ordvar[i], totalItemSum);
                }
                
                for (int i = 0; i < aisles.size(); i++) {
                    objetivo.setCoefficient(aisvar[i], -alpha);
                }

                objetivo.setMaximization();

                solver.setTimeLimit((getRemainingTime(stopWatch)-20)/(BIN_ITER-c) * 1000);
                
                var constraints = solver.constraints();

                var resultStatus = solver.solve();

                if(resultStatus == MPSolver.ResultStatus.INFEASIBLE){
                    System.out.println("The solver could not solve the problem.");
                    break;
                }

                if(resultStatus == MPSolver.ResultStatus.OPTIMAL && Math.abs(objetivo.value()) < epsilon){
                    System.out.println("Optimal solution found.");
                    break;
                }

                if (resultStatus != MPSolver.ResultStatus.OPTIMAL) {
                    System.out.println("The problem does not have an optimal solution!");
                    if (resultStatus == MPSolver.ResultStatus.FEASIBLE) {
                        System.out.println("A potentially suboptimal solution was found");
                    } else {
                        System.out.println("The solver could not solve the problem.");
                    }
                }

                System.out.println("Solution:");
                System.out.println("Objective value = " + objetivo.value());
                // [END print_solution]

                // [START advanced]
                System.out.println("Advanced usage:");
                System.out.println("Problem solved in " + solver.wallTime() + " milliseconds");
                System.out.println("Problem solved in " + solver.iterations() + " iterations");


                double g = 0;
                for(int a=0;a<aisles.size();a++){
                    if(aisvar[a].solutionValue() > 0.5){ g++; }
                }
                
                alpha = alpha + objetivo.value() / g;
                
                bestOrders.clear();
                bestAisles.clear();

                for (int i = 0; i < orders.size(); i++) {
                    if(ordvar[i].solutionValue() > 0.5)
                        bestOrders.add(i);
                }
                for (int i = 0; i < aisles.size(); i++) {
                    if(aisvar[i].solutionValue() > 0.5)
                        bestAisles.add(i);
                }
            
            }
        }catch(Exception e){

        }

        return new ChallengeSolution(bestOrders,bestAisles);
    }
}
