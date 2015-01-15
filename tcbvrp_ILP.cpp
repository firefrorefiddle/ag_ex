#include "tcbvrp_ILP.h"

tcbvrp_ILP::tcbvrp_ILP( Instance& _instance, string _model_type) :
instance( _instance ), model_type( _model_type )
{
	//Number of stations + depot
	n = instance.n;
	//Number of edges
	a = instance.nArcs;
	//Max. Number of vehicles
	m = instance.m;
	//Max. time limit
	T = instance.T;
}

tcbvrp_ILP::~tcbvrp_ILP()
{
	// free CPLEX resources
	cplex.end();
	model.end();
	env.end();
}

void tcbvrp_ILP::solve()
{
	try {
		// initialize CPLEX solver
		env = IloEnv();
		model = IloModel( env );

		// add model-specific constraints
		if( model_type == "scf" )
			modelSCF();
		else if( model_type == "mcf" )
			modelMCF();
		else if( model_type == "mtz" )
			modelMTZ();

		// build model
		cplex = IloCplex( model );

		// export model to a text file
		cplex.exportModel( "model.lp" );

		// set parameters
		setCPLEXParameters();

		// solve model
		cout << "Calling CPLEX solve ...\n";
		cplex.solve();
		cout << "CPLEX finished." << "\n\n";
		cout << "CPLEX status: " << cplex.getStatus() << "\n";
		cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << "\n";
		cout << "Objective value: " << cplex.getObjValue() << "\n";
		cout << "CPU time: " << Tools::CPUtime() << "\n\n";

		if( false ) {
			// TODO optionally output the values of the variables
		}
	}
	catch( IloException& e ) {
		cerr << "tcbvrp_ILP: exception " << e << "\n";
		exit( -1 );
	}
	catch( ... ) {
		cerr << "tcbvrp_ILP: unknown exception.\n";
		exit( -1 );
	}
}

// ----- private methods -----------------------------------------------

void tcbvrp_ILP::setCPLEXParameters()
{
	// print every line of node-log and give more details
	cplex.setParam( IloCplex::MIPInterval, 1 );
	cplex.setParam( IloCplex::MIPDisplay, 2 );

	// deactivate CPLEX general-purpose cuts
	//	cplex.setParam( IloCplex::EachCutLim, 0 );
	//	cplex.setParam( IloCplex::FracCuts, -1 );

	// only use a single thread
	cplex.setParam( IloCplex::Threads, 1 );
	// set time limit for cplex (in seconds)
	cplex.setParam( IloCplex::TiLim, 3600);
}


void tcbvrp_ILP::modelSCF()
{
  // our variables are indexed x[k][i][j]
  // where k is the tour, i the start node, 
  // j the target node and x the flow from i 
  // to j in tour k.
  
  // multi dimensional IloArrays are possible,
  // but complicated, so a flat index seems
  // easier.
  IloIntVarArray flow(env, n * n * m);
  
  // x is a corresponding array for the costs
  // if flow[k,i,j] > 0 -> x[k,i,j] = t[i,j]
  IloIntVarArray x(env, n * n * m);

  // cost function
  IloExpr totalCosts(env);

  // initialize variables and cost function  
  for (u_int i=0; i<n; i++) 
    {
      for (u_int j=0; j<n; j++) 
	{
	  for (u_int k=0; k<m; k++)
	    {
	      stringstream flowname;
	      flowname << "f_" << i+1 << "_" << j+1 << "_" << k+1;
	      stringstream xname;
	      xname << "x_" << i+1 << "_" << j+1 << "_" << k+1;

	      flow[index3(i,j,k)] = IloIntVar(env, 0, n-1, flowname.str().c_str());

	      //	      IloIntArray xvalues(env, 2);
	      //	      xvalues[0] = 0;
	      //	      xvalues[1] = instance.getDistance(i, j);
	      //	      x[index3(i,j,k)] = IloIntVar(env, xvalues, xname.str().c_str());

	      totalCosts += flow[index3(i,j,k)];
	    }
	}
    }

  model.add(IloMinimize(env, totalCosts));
 
  // totalCosts.end();

	//++++++++++++++++++++++++++++++++++++++++++
	//TODO build single commodity flow model
	//++++++++++++++++++++++++++++++++++++++++++
}

void tcbvrp_ILP::modelMCF()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build multi commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
}

void tcbvrp_ILP::modelMTZ()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build Miller-Tucker-Zemlin model
	// ++++++++++++++++++++++++++++++++++++++++++
}




