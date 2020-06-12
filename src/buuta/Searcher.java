package buuta;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

import localsearch.domainspecific.vehiclerouting.vrp.CBLSVR;
import localsearch.domainspecific.vehiclerouting.vrp.VRManager;
import localsearch.domainspecific.vehiclerouting.vrp.ValueRoutesVR;
import localsearch.domainspecific.vehiclerouting.vrp.VarRoutesVR;
import localsearch.domainspecific.vehiclerouting.vrp.entities.LexMultiValues;
import localsearch.domainspecific.vehiclerouting.vrp.entities.Point;
import localsearch.domainspecific.vehiclerouting.vrp.functions.LexMultiFunctions;
import localsearch.domainspecific.vehiclerouting.vrp.moves.IVRMove;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.INeighborhoodExplorer;
import localsearch.domainspecific.vehiclerouting.vrp.search.ISearch;
import localsearch.domainspecific.vehiclerouting.vrp.search.NeighbohoodExplorerManager;
import localsearch.domainspecific.vehiclerouting.vrp.search.Neighborhood;

public class Searcher implements ISearch{
	
	public VRManager mgr;
	public VarRoutesVR XR; 
	public LexMultiFunctions F;
	public NeighbohoodExplorerManager NEM;
	public int maxStable;
	
	public ValueRoutesVR bestSolution;
	public LexMultiValues bestValue;
	public int currentIter, nic;
	public boolean verbose=true;
	
	private Random R = new Random(0);

	
		
	public Searcher(VRManager mgr, LexMultiFunctions F, ArrayList<INeighborhoodExplorer> neighborhoodExplorer){
		this.mgr = mgr;
		this.XR = mgr.getVarRoutesVR();
		this.F = F;
		//this.neighborhoodExplorer = neighborhoodExplorer;
		NEM = new NeighbohoodExplorerManager(neighborhoodExplorer);
		this.maxStable = 100;
	}
	public Searcher(VRManager mgr) {
		this.mgr = mgr;
		this.XR = mgr.getVarRoutesVR();
		this.maxStable = 100;
		NEM = new NeighbohoodExplorerManager();
	}
	
//	public void generateInitialSolution() {
//		for (Point p : XR.getClientPoints()) {
//			ArrayList<Point> L = XR.collectCurrentClientAndStartPointsOnRoute();
//			Point q = L.get(R.nextInt(L.size()));
//			mgr.performAddOnePoint(p, q);
//		}
//	}
	
	public void generateInitialSolution() {
		mgr.performRemoveAllClientPoints();
		System.out.println("After removing all client points, XR = " + XR.toString());
		HashSet<Point> S = new HashSet<Point>();
		for (Point p : XR.getClientPoints()) {
			S.add(p);
		}
		while (S.size() > 0) {
			LexMultiValues eval = new LexMultiValues();
			eval.fill(1, CBLSVR.MAX_INT);
			Point sel_q = null;
			Point sel_p = null;
			ArrayList<PairPoints> Cand = new ArrayList<PairPoints>();
			for (Point p : S) {
				ArrayList<Point> L = XR
						.collectCurrentClientAndStartPointsOnRoute();
				for (Point q : L) {
					LexMultiValues e = F.evaluateAddOnePoint(p, q);
					if (e.lt(eval)) {
						Cand.clear();
						Cand.add(new PairPoints(p,q));
						eval = e;
					}else if(e.eq(eval)){
						Cand.add(new PairPoints(p,q));
					}
				}
			}
			PairPoints PP = Cand.get(R.nextInt(Cand.size()));
			sel_p = PP.x; sel_q = PP.y;
			mgr.performAddOnePoint(sel_p, sel_q);
			S.remove(sel_p);
		}
		System.out.println("generateInitialSolution finished, cost = "
				+ F.getValues().toString());
		
		//mgr.performRemoveAllClientPoints();
		//System.exit(-1);
	}
	
	
	public void search(int maxIter, int maxTime){
		bestSolution = new ValueRoutesVR(XR);
		currentIter = 0;
		generateInitialSolution();
		nic = 0;
		Neighborhood N = new Neighborhood(mgr);
		bestValue = new LexMultiValues(F.getValues());
		updateBest();
		
		if(verbose) System.out.println("Init bestValue = " + bestValue.toString());
		if(verbose) System.out.println(XR.toString());
		while (currentIter < maxIter) {
			N.clear();
			LexMultiValues bestEval = new LexMultiValues();
			bestEval.fill(F.size(), 0);
						
			NEM.exploreNeighborhoodsFirstImprovement(N, bestEval, currentIter);
						
			if (N.hasMove()) {
				IVRMove m = N.getAMove();
				m.move();
				
				if(verbose) System.out.println("Step " + currentIter + ", F = " + F.getValues().toString() + 
						", best = " + bestValue.toString());
				
				processNeighbor();
				
				
			} else {
				if(verbose) System.out.println("search --> no move available, break");
				nic++;
				if (nic > maxStable) {
					restart(currentIter);
				}
				
			}
			
			currentIter++;

		}

		XR.setValue(bestSolution);	
	}
	
	public void updateBest() {
		bestValue.set(F.getValues());
		bestSolution.store();	
		nic = 0;
	}
	
	public void processNeighbor(){
		if(F.getValues().lt(bestValue)){
			updateBest();
		}else{
			nic++;
			if(nic > maxStable){
				restart(currentIter);
			}
		}

	}
	
	public void restart(int currentIter){
		//XR.setRandom();
		//generateInitialSolution();
		perturb(XR.getNbClients());
		if(F.getValues().lt(bestValue)){
			updateBest();
		}
		nic = 0;
		NEM.restart(currentIter);
	}
	
	public void perturb(int nbSteps){
		for(int k = 1; k <= nbSteps; k++){
			ArrayList<Point> P = XR.collectClientPointsOnRoutes();
			if(P.size() >= 2){
				for(int i = 1; i <= 10; i++){
					Point x = P.get(R.nextInt(P.size()));
					Point y = P.get(R.nextInt(P.size()));
					
					if(x != y && XR.checkPerformOnePointMove(x, y)){
						mgr.performOnePointMove(x, y);
						break;
					}
					
				}
			}
		}
	}
	
	
	public int getCurrentIteration() {
		// TODO Auto-generated method stub
		return 0;
	}

	public ValueRoutesVR getIncumbent() {
		// TODO Auto-generated method stub
		return null;
	}

	public LexMultiValues getIncumbentValue() {
		// TODO Auto-generated method stub
		return null;
	}

	public void setNeighborhoodExplorer(ArrayList<INeighborhoodExplorer> nE) {
		// TODO Auto-generated method stub
		NEM.setNeighborhoodExplorers(nE);
	}
	public void setObjectiveFunction(LexMultiFunctions f2) {
		// TODO Auto-generated method stub
		this.F = f2;
	}
	public void setMaxStable(int i) {
		// TODO Auto-generated method stub
		this.maxStable = i;
	}


}
