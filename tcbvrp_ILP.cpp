#include "tcbvrp_ILP.h"

#include <vector>
#include <fstream>

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
    // the basic variables
    x = IloIntVarArray(env, n * n * m);
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

        if( true )
        {
            ofstream outfile("values.lp");
            IloNumArray values(env); // , x.getSize());
            cplex.getValues(values, x);

            for (u_int k=0; k<m; k++)
            {
                outfile << " *** Tour *** " << k << ":" << endl;

                for (u_int i=0; i<n; i++)
                {
                    for (u_int j=0; j<n; j++)
                    {
                        outfile << values[index3(i,j,k)] << " ";
                    }
                    outfile << endl;
                }

                outfile << endl << endl;
            }

            outfile << "Costs: " << cplex.getObjValue() << endl;

            outfile.close();
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
    //  IloIntVarArray flow(env, n * n * m);

    // cost function
    IloExpr totalCosts(env);

    // *** initialize variables and cost function  ***
    for (u_int i=0; i<n; i++)
    {
        for (u_int j=0; j<n; j++)
        {
            for (u_int k=0; k<m; k++)
            {
                //	      stringstream flowname;
                //	      flowname << "f_" << i+1 << "_" << j+1 << "_" << k+1;
                stringstream xname;
                xname << "x_" << i+1 << "_" << j+1 << "_" << k+1;

                //	      flow[index3(i,j,k)] = IloIntVar(env, 0, n-1, flowname.str().c_str());
                x[index3(i,j,k)] = IloIntVar(env, 0, 1, xname.str().c_str());

                totalCosts += (x[index3(i,j,k)] * instance.getDistance(i,j));
            }
        }
    }

    model.add(x);
    model.add(IloMinimize(env, totalCosts));

    // *** add constraints ***

    // Sum_im x_mij = 1 forall j in D
    //   i.e. each demand node is visited exactly once

    IloArray<IloExpr> eachDemandOnce(env, n);

    for(vector<int>::const_iterator iter = instance.beginDemandNodes();
            iter != instance.endDemandNodes();
            ++iter)
    {
        eachDemandOnce[*iter-1] = IloExpr(env);
        for(u_int i=0; i<n; ++i)
        {
            for(u_int k=0; k<m; ++k)
            {
                eachDemandOnce[*iter-1] += x[index3(i,*iter-1,k)];
            }
        }
        model.add(eachDemandOnce[*iter-1] == 1);
    }

    // Sum_im x_mij <= 1 forall j in S
    //   i.e. each supply node is visited at most once

    IloArray<IloExpr> eachSupplyOnce(env, n);

    for(vector<int>::const_iterator iter = instance.beginSupplyNodes();
            iter != instance.endSupplyNodes();
            ++iter)
    {
        eachSupplyOnce[*iter-1] = IloExpr(env);
        for(u_int i=0; i<n; ++i)
        {
            for(u_int k=0; k<m; ++k)
            {
                eachSupplyOnce[*iter-1] += x[index3(i,*iter-1,k)];
            }
        }
        model.add(eachSupplyOnce[*iter-1] <= 1);
    }

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




