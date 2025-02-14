package org.sbpo2025.challenge;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.apache.commons.lang3.time.StopWatch;

public class HeuristicChallengeSolver extends ChallengeSolver {
    private final long MAX_RUNTIME = 540; // 9 minutes
    private ArrayList<Integer> ordersIndexes = new ArrayList<Integer>();
    private ArrayList<Integer> aislesIndexes = new ArrayList<Integer>();
        

    public HeuristicChallengeSolver(List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems,
            int waveSizeLB, int waveSizeUB) {
        super(orders, aisles, nItems, waveSizeLB, waveSizeUB);

        for(int i = 0 ; i < orders.size() ; i++){
            ordersIndexes.add(i);
        }

        for(int i = 0 ; i < aisles.size() ; i++){
            aislesIndexes.add(i);
        }
    }
    
    @Override
    public ChallengeSolution solve(StopWatch stopWatch) {
        var bestNumberOfItems = 0;
        Set<Integer> bestOrders = new TreeSet<>();
        Set<Integer> bestAisles = new TreeSet<>();

        for(int iteration = 0 ; iteration < 100 ; iteration++){
            var takedOrders = new TreeSet<Integer>();

            if(stopWatch.getDuration().getSeconds() > MAX_RUNTIME){
                break;
            }

            for(var orderNumber: ordersIndexes){
                takedOrders.add(orderNumber);
                
                var ailes = bestAislesToOrders(takedOrders);
                var currentNumberOfItems = getOrdersItemsSum(takedOrders);

                if(ailes.size() == 0 || currentNumberOfItems > waveSizeUB){
                    takedOrders.remove(orderNumber);
                }else if(
                    currentNumberOfItems >= waveSizeLB && (
                        bestNumberOfItems == 0 ||
                        currentNumberOfItems * bestAisles.size() > bestNumberOfItems * ailes.size()
                    )
                ){
                        bestNumberOfItems = currentNumberOfItems;
                        bestOrders = (TreeSet<Integer>)takedOrders.clone();
                        bestAisles = (TreeSet<Integer>)ailes.clone();
                }
            }
            
            Collections.shuffle(ordersIndexes);
        }

        return new ChallengeSolution(bestOrders, bestAisles);
    }

    private int getOrdersItemsSum(Set<Integer> selectedOrders){
        var sum = 0;

        for(var orderNumber: selectedOrders){
            for(var entry: orders.get(orderNumber).entrySet()){
                sum += entry.getValue();
            }
        }

        return sum;
    }

    private TreeSet<Integer> bestAislesToOrders(TreeSet<Integer> selectedOrders){
        var orderItemsSum = new Integer[nItems];

        for(int i = 0 ; i < nItems ; i++){
            orderItemsSum[i] = 0;
        }

        for(var orderNumber: selectedOrders){
            for(var entry: orders.get(orderNumber).entrySet()){
                orderItemsSum[entry.getKey()] += entry.getValue();
            }
        }

        var bestAnswer = new TreeSet<Integer>();

        for(int iteration = 0 ; iteration < 10 ; iteration++){
            var aisleItemsSum = new Integer[nItems];
            var takedAisles = new TreeSet<Integer>();

            for(int i = 0 ; i < nItems ; i++){
                aisleItemsSum[i] = 0;
            }

            for(int aisleNumber = 0 ; aisleNumber < aisles.size() ; aisleNumber++){
                takedAisles.add(aisleNumber);
    
                for(var entry: aisles.get(aisleNumber).entrySet()){
                    aisleItemsSum[entry.getKey()] += entry.getValue();
                }
            }

            for(int item = 0 ; item < nItems ; item++){
                if(aisleItemsSum[item] < orderItemsSum[item]){//Nenhuma solucao possivel, retorne vazio
                    return new TreeSet<Integer>();
                }
            }

            for(var aisleNumber: aislesIndexes){
                var canRemove = true;

                for(var entry: aisles.get(aisleNumber).entrySet()){
                    if(aisleItemsSum[entry.getKey()] - entry.getValue() < orderItemsSum[entry.getKey()]){
                        canRemove = false;
                        break;
                    }
                }
                
                if(canRemove){
                    for(var entry: aisles.get(aisleNumber).entrySet()){
                        aisleItemsSum[entry.getKey()] -= entry.getValue();
                    }

                    takedAisles.remove(aisleNumber);
                }
            }

            if(bestAnswer.size() == 0 || takedAisles.size() < bestAnswer.size()){
                bestAnswer = (TreeSet<Integer>) takedAisles.clone();
            }
            
            Collections.shuffle(aislesIndexes);
        }

        return bestAnswer;
    }
}
