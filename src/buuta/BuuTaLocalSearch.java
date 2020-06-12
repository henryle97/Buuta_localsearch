package buuta;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

//import localsearch.domainspecific.vehiclerouting.apps.minmaxvrp.MMSearch;
import localsearch.domainspecific.vehiclerouting.vrp.IFunctionVR;
import localsearch.domainspecific.vehiclerouting.vrp.VRManager;
import localsearch.domainspecific.vehiclerouting.vrp.VarRoutesVR;
import localsearch.domainspecific.vehiclerouting.vrp.entities.ArcWeightsManager;
import localsearch.domainspecific.vehiclerouting.vrp.entities.Point;
import localsearch.domainspecific.vehiclerouting.vrp.functions.AccumulatedEdgeWeightsOnPathVR;
import localsearch.domainspecific.vehiclerouting.vrp.functions.LexMultiFunctions;
import localsearch.domainspecific.vehiclerouting.vrp.functions.MaxVR;
import localsearch.domainspecific.vehiclerouting.vrp.functions.TotalCostVR;
import localsearch.domainspecific.vehiclerouting.vrp.invariants.AccumulatedWeightEdgesVR;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.GreedyCrossExchangeMoveExplorer;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.GreedyOnePointMoveExplorer;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.GreedyThreeOptMove1Explorer;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.GreedyTwoOptMove1Explorer;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.GreedyTwoPointsMoveExplorer;
import localsearch.domainspecific.vehiclerouting.vrp.neighborhoodexploration.INeighborhoodExplorer;

class PairPoints{
	public Point x;
	public Point y;
	public PairPoints(Point x, Point y){
		this.x = x; this.y = y;
	}
}

public class BuuTaLocalSearch {
	private ArcWeightsManager awm;
	private VRManager mgr;
	private VarRoutesVR XR;
	private ArrayList<Point> startPoints;
	private ArrayList<Point> endPoints;
	private ArrayList<Point> clientPoints;
	private ArrayList<Point> allPoints;
	private IFunctionVR[] costRoute;
	public IFunctionVR min_max_route_cost;
	public IFunctionVR total_distance_cost;
	private LexMultiFunctions F;
	
	public void readData(String filePath ) {
		try {
			
			// Read file 
			Scanner in = new Scanner(new File(filePath));
			int K;// number of vehicles
			K = in.nextInt();
			ArrayList<Integer> x = new ArrayList<Integer>();
			ArrayList<Integer> y = new ArrayList<Integer>();
			while (true) {
				int id = in.nextInt();
				if (id == -1)
					break;
				int cx = in.nextInt();
				int cy = in.nextInt();
				x.add(cx);
				y.add(cy);
			}
			in.close();
			
			clientPoints = new ArrayList<Point>();
			startPoints = new ArrayList<Point>();
			endPoints = new ArrayList<Point>();
			allPoints = new ArrayList<Point>();

			for (int i = 1; i < x.size(); i++) {
				Point p = new Point(i);
				clientPoints.add(p);
				allPoints.add(p);
			}
			int id = x.size() - 1;
			for (int k = 0; k < K; k++) {
				id++;
				Point s = new Point(id);
				startPoints.add(s);
				allPoints.add(s);
				id++;
				Point t = new Point(id);
				endPoints.add(t);
				allPoints.add(t);
			}

			awm = new ArcWeightsManager(allPoints);

			for (int i = 0; i < clientPoints.size(); i++) {
				Point pi = clientPoints.get(i);
				int xi = x.get(i + 1);
				int yi = y.get(i + 1);
				for (int j = 0; j < clientPoints.size(); j++) {
					Point pj = clientPoints.get(j);
					int xj = x.get(j + 1);
					int yj = y.get(j + 1);
					double w = Math.sqrt(Math.pow(xi - xj, 2) + Math.pow(yi - yj, 2));
					awm.setWeight(pi, pj, w);
				}
			}

			for (int i = 0; i < clientPoints.size(); i++) {
				int xi = x.get(i + 1);
				int yi = y.get(i + 1);
				Point p = clientPoints.get(i);

				for (int k = 0; k < startPoints.size(); k++) {
					Point s = startPoints.get(k);
					Point t = endPoints.get(k);
					int xk = x.get(0);
					int yk = y.get(0);
					double w = Math.sqrt(Math.pow(xi - xk, 2) + Math.pow(yi - yk, 2));
					awm.setWeight(p, s, w);
					awm.setWeight(p, t, w);
					awm.setWeight(s, p, w);
					awm.setWeight(t, p, w);
				}
			}
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}
	
	public void stateModel() {
		System.out.println("===== STATE MODEL ====");
		mgr = new VRManager();
		XR = new VarRoutesVR(mgr);
		for (int k = 0; k < startPoints.size(); k++) {
			XR.addRoute(startPoints.get(k), endPoints.get(k));
		}
		for (Point p : clientPoints) {
			XR.addClientPoint(p);
		}

		AccumulatedWeightEdgesVR awe = new AccumulatedWeightEdgesVR(XR, awm);
		costRoute = new IFunctionVR[startPoints.size()];
		for (int k = 0; k < XR.getNbRoutes(); k++) {
			costRoute[k] = new AccumulatedEdgeWeightsOnPathVR(awe,
					XR.endPoint(k + 1));
		}
		
		// Objective 
		min_max_route_cost = new MaxVR(costRoute);
		total_distance_cost = new TotalCostVR(XR, awm);
		
		F = new LexMultiFunctions();
		F.add(min_max_route_cost);
		F.add(total_distance_cost);

		mgr.close();

	}
	
	public void search() {
		ArrayList<INeighborhoodExplorer> NE = new ArrayList<INeighborhoodExplorer>();
//		NE.add(new GreedyOnePointMoveExplorer(XR, F));
		NE.add(new GreedyTwoOptMove1Explorer(XR, F));
//		NE.add(new GreedyThreeOptMove1Explorer(XR, F));
		NE.add(new GreedyCrossExchangeMoveExplorer(XR, F));

		
//		Searcher searcher = new Searcher(mgr);
		MMSearch searcher =  new MMSearch(mgr);
		searcher.setNeighborhoodExplorer(NE);
		searcher.setObjectiveFunction(F);
		searcher.setMaxStable(20);

		searcher.search(1000, 100);
	}
	
	
	
	
	public void printSolution() {
		System.out.println(XR.toString());
		for (int k = 1; k <= XR.getNbRoutes(); k++)
			System.out.println("cost[" + k + "] = "
					+ costRoute[k - 1].getValue());
		System.out.println("Min long route cost: " + min_max_route_cost.getValue());
		System.out.println("Total Cost = " + total_distance_cost.getValue());
	}
	
	public static void main(String[] args) {
		BuuTaLocalSearch app = new BuuTaLocalSearch();
//		String[] test_files = {"data_10", "data_50", "data_100", "data_200", "data_300", "data_500", "data_1000"} ;
		String[] test_files = {"data_4"} ;
		ArrayList<String> res = new ArrayList<String>();
		for (String file: test_files) {
			app.readData("data/"+ file);
			app.stateModel();
			double t0 = System.currentTimeMillis();

			app.search();
			double time  = System.currentTimeMillis() - t0;

//			time_to_best = time_to_best * 0.001;
			app.printSolution();
			
			System.out.println("Total time:" + time/1000);
			String result = "File:" + file + ", Min length: " + app.min_max_route_cost.getValue() + ", cost=" + app.total_distance_cost.getValue()
					+ ", time=" + time/1000;
			res.add(result);
			for (String r : res) {
				System.out.println(r);

			}
		}
		for (String r : res) {
			System.out.println(r);

		}
			
	}
	
}
