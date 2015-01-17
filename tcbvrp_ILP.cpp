#include "tcbvrp_ILP.h"

#include <vector>
#include <fstream>
#include <cmath>

tcbvrp_ILP::tcbvrp_ILP(Instance &_instance, string _model_type) :
    instance(_instance), model_type(_model_type)
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
    try
    {
        // initialize CPLEX solver
        env = IloEnv();
        model = IloModel(env);

        // add model-specific constraints
        if (model_type == "scf")
            modelSCF();
        else if (model_type == "mcf")
            modelMCF();
        else if (model_type == "mtz")
            modelMTZ();

        // build model
        cplex = IloCplex(model);

        // export model to a text file
        cplex.exportModel("model.lp");

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

        if (true)
        {
            ofstream outfile("values.lp");
            IloNumArray values(env); // , x.getSize());
            cplex.getValues(values, x);

            for (u_int k = 0; k < m; k++)
            {
                int tourCost = 0;

                outfile << " *** Tour *** " << k << ":" << endl;

                outfile << "   ";
                for (u_int i = 0; i < n; i++)
                {
                    outfile << setw(2) << i << " ";
                }
                outfile << endl;

                for (u_int i = 0; i < n; i++)
                {
                    outfile << setw(2) << i << " ";

                    for (u_int j = 0; j < n; j++)
                    {
                        int value = round(values[index3(i, j, k)]) * instance.getDistance(i, j);

                        tourCost += value;

                        outfile << setw(2) << (value == -0 ? 0 : value) << " ";
                    }
                    outfile << endl;
                }

                outfile << endl << "Tour costs: " << tourCost << endl << endl;
            }

            outfile << "Costs: " << cplex.getObjValue() << endl;
            outfile.close();

            ofstream objfile("objective.lp");
            objfile << cplex.getObjValue() << endl;
            objfile.close();
        }
    }
    catch (IloException &e)
    {
        cerr << "tcbvrp_ILP: exception " << e << "\n";
        exit(-1);
    }
    catch (...)
    {
        cerr << "tcbvrp_ILP: unknown exception.\n";
        exit(-1);
    }
}

// ----- private methods -----------------------------------------------

void tcbvrp_ILP::setCPLEXParameters()
{
    // print every line of node-log and give more details
    cplex.setParam(IloCplex::MIPInterval, 1);
    cplex.setParam(IloCplex::MIPDisplay, 2);

    // deactivate CPLEX general-purpose cuts
    //  cplex.setParam( IloCplex::EachCutLim, 0 );
    //  cplex.setParam( IloCplex::FracCuts, -1 );

    // only use a single thread
    cplex.setParam(IloCplex::Threads, 1);
    // set time limit for cplex (in seconds)
    cplex.setParam(IloCplex::TiLim, 3600);
}

void tcbvrp_ILP::modelGeneral()
{

    // cost function
    IloExpr totalCosts(env);

    // *** initialize variables and cost function  ***
    for (u_int i = 0; i < n; i++)
    {
        for (u_int j = 0; j < n; j++)
        {

            // if this is not an allowed arc, then we will just give 0 as a maximum
            // var value

            // how UGLY! go replace it with something nicer...!
            int iSupply = instance.isSupplyNode(i);
            int iDemand = instance.isDemandNode(i);
            int iDepot  = (iSupply + iDemand) == 0 ? 1 : 0;
            int jSupply = instance.isSupplyNode(j);
            int jDemand = instance.isDemandNode(j);
            int jDepot  = (jSupply + jDemand) == 0 ? 1 : 0;

            int max;

            switch ((iSupply << 5) + (iDemand << 4) + (iDepot << 3) +
                    (jSupply << 2) + (jDemand << 1) + jDepot)
            {
                case (1 << 5) + (1 << 1): // supply -> demand
                    max = 1;
                    break;
                case (1 << 4) + (1 << 2): // demand -> supply
                    max = 1;
                    break;
                case (1 << 4) + 1:        // demand -> depot
                    max = 1;
                    break;
                case (1 << 3) + (1 << 2): // depot -> supply
                    max = 1;
                    break;
                case (1 << 3) + 1:        // depot -> depot
                    max = 1;
                    break;
                default:
                    max = 0;
            }

            for (u_int k = 0; k < m; k++)
            {
                stringstream xname;
                xname << "x_" << i << "_" << j << "_" << k;

                x[index3(i, j, k)] = IloIntVar(env, 0, max, xname.str().c_str());

                totalCosts += (x[index3(i, j, k)] * instance.getDistance(i, j));
            }
        }
    }

    model.add(x);
    model.add(IloMinimize(env, totalCosts));

    // *** add constraints ***

    // Sum_im x_mij = 1 forall j in D
    //   i.e. each demand node is visited exactly once

    IloArray<IloExpr> eachDemandOnce(env, n);

    for (vector<int>::const_iterator iter = instance.beginDemandNodes();
         iter != instance.endDemandNodes();
         ++iter)
    {
        eachDemandOnce[*iter] = IloExpr(env);
        for (u_int i = 0; i < n; ++i)
        {
            for (u_int k = 0; k < m; ++k)
            {
                eachDemandOnce[*iter] += x[index3(i, *iter, k)];
            }
        }
        model.add(eachDemandOnce[*iter] == 1);
    }

    // also visit the depot once each tour
    for (u_int k = 0; k < m; ++k)
    {
        IloExpr visitDepotOnce(env);
        for (u_int i = 0; i < n; ++i)
        {
            visitDepotOnce += x[index3(i, 0, k)];
        }
        model.add(visitDepotOnce == 1);
    }

    // Sum_im x_mij <= 1 forall j in S
    //   i.e. each supply node is visited at most once

    IloArray<IloExpr> eachSupplyAtMostOnce(env, n);

    for (vector<int>::const_iterator iter = instance.beginSupplyNodes();
         iter != instance.endSupplyNodes();
         ++iter)
    {
        eachSupplyAtMostOnce[*iter] = IloExpr(env);
        for (u_int i = 0; i < n; ++i)
        {
            for (u_int k = 0; k < m; ++k)
            {
                eachSupplyAtMostOnce[*iter] += x[index3(i, *iter, k)];
            }
        }
        model.add(eachSupplyAtMostOnce[*iter] <= 1);
    }

    // Sum_x_mij = Sum_x_mjk forall mj

    for (u_int j = 0; j < n; ++j)
    {
        for (u_int k = 0; k < m; ++k)
        {
            IloExpr incomingConnections(env);

            for (u_int i = 0; i < n; ++i)
            {
                incomingConnections += x[index3(i, j, k)];
            }

            IloExpr outgoingConnections(env);

            for (u_int l = 0; l < n; ++l)
            {
                outgoingConnections += x[index3(j, l, k)];
            }

            model.add(incomingConnections == outgoingConnections);
        }
    }

    // Sum_x_mij <= t forall m
    //  i.e. restrict length of each subtour
    for (u_int k = 0; k < m; ++k)
    {
        IloExpr tourLength(env);
        for (u_int i = 0; i < n; ++i)
        {
            for (u_int j = 0; j < n; ++j)
            {
                tourLength += (x[index3(i, j, k)] * instance.getDistance(i, j));
            }
        }

        model.add(tourLength <= (int)T);
    }

}

void tcbvrp_ILP::modelSCF()
{

    cout << "Using SCF Algorithm" << endl;

    modelGeneral();

    IloIntVarArray flow(env, n * n * m);

    for (u_int i = 0; i < n; ++i)
    {
        for (u_int j = 0; j < n; ++j)
        {
            for (u_int k = 0; k < m; ++k)
            {
                stringstream flowname;
                flowname << "f_" << i << "_" << j << "_" << k;
                flow[index3(i, j, k)] = IloIntVar(env, 0, n - 1, flowname.str().c_str());
            }
        }
    }

    IloExpr sumDepot(env);

    for (u_int k = 0; k < m; ++k)
    {
        //
        // ** n-1 going out from the depot
        //

        for (u_int j = 1; j < n; ++j)
        {
            sumDepot += flow[index3(0, j, k)];
        }

        //
        // ** outgoing flow matches incoming flow
        //
        for (u_int j = 1; j < n; j++)
        {

            IloIntVar any(env, 0, 1);

            IloExpr sumIncoming(env);
            for (u_int i = 0; i < n; i++)
            {
                if (i != j)
                {
                    sumIncoming += flow[index3(i, j, k)];
                    model.add(any >= x[index3(i, j, k)]);
                }
            }

            IloExpr sumOutgoing(env);
            for (u_int l = 0; l < n; l++)
            {
                if (j != l)
                {
                    sumOutgoing += flow[index3(j, l, k)];
                }
            }

            if (instance.isDemandNode(j))
            {
                model.add((sumIncoming - sumOutgoing) == any);
            }
            else
            {
                model.add((sumIncoming - sumOutgoing) == 0);
            }

        }

        //
        // ** flow at most n-1 or 0
        //
        for (u_int i = 0; i < n; i++)
        {
            for (u_int j = 0; j < n; j++)
            {
                if (i == j) continue;
                model.add(flow[index3(i, j, k)] >= 0);
                model.add(flow[index3(i, j, k)] <= (x[index3(i, j, k)] * ((int) n - 1)));
            }
        }
    }

    model.add(sumDepot == instance.nDemandNodes());

}

void tcbvrp_ILP::modelMCF()
{
    cout << "Using MCF Algorithm" << endl;

    modelGeneral();

    for (u_int c = 0; c < n; c++)
    {

        if (!instance.isDemandNode(c))
        {
            continue;
        }

        IloIntVarArray flow(env, n * n * m);

        for (u_int i = 0; i < n; ++i)
        {
            for (u_int j = 0; j < n; ++j)
            {
                for (u_int k = 0; k < m; ++k)
                {
                    stringstream flowname;
                    flowname << "f_" << i << "_" << j << "_" << k;
                    flow[index3(i, j, k)] = IloIntVar(env, 0, n - 1, flowname.str().c_str());
                }
            }
        }

        IloExpr sumDepotOut(env);
        IloExpr sumDepotIn(env);

        for (u_int k = 0; k < m; ++k)
        {
            //
            // ** n-1 going out from the depot
            //

            for (u_int j = 1; j < n; ++j)
            {
                sumDepotOut += flow[index3(0, j, k)];
            }

            for (u_int i = 1; i < n; ++i)
            {
                sumDepotIn += flow[index3(i, 0, k)];
            }

            //
            // ** outgoing flow matches incoming flow
            //
            for (u_int j = 1; j < n; j++)
            {

                IloIntVar any(env, 0, 1);

                IloExpr sumIncoming(env);
                for (u_int i = 0; i < n; i++)
                {
                    if (i != j)
                    {
                        sumIncoming += flow[index3(i, j, k)];
                        model.add(any >= x[index3(i, j, k)]);
                    }
                }

                IloExpr sumOutgoing(env);
                for (u_int l = 0; l < n; l++)
                {
                    if (j != l)
                    {
                        sumOutgoing += flow[index3(j, l, k)];
                    }
                }

                if (j == c)
                {
                    model.add((sumIncoming - sumOutgoing) == any);
                }
                else
                {
                    model.add((sumIncoming - sumOutgoing) == 0);
                }

            }

            //
            // ** flow at most n-1 or 0
            //
            for (u_int i = 0; i < n; i++)
            {
                for (u_int j = 0; j < n; j++)
                {
                    if (i == j) continue;
                    model.add(flow[index3(i, j, k)] >= 0);
                    model.add(flow[index3(i, j, k)] <= (x[index3(i, j, k)] * ((int) n - 1)));
                }
            }
        }

        model.add((sumDepotOut - sumDepotIn) == 1);
    }
}

void tcbvrp_ILP::modelMTZ()
{
    cout << "Using MTZ Algorithm" << endl;

    modelGeneral();

    // we use just the same numbering as in the standard TSP problem,
    // only that we additionally make sure that each number for a city
    // visited in tour k is lower than all the numbers of cities visited
    // in any tour k+j (j in {1..m-k}).

    IloIntVarArray u(env, n);

    for (u_int i = 0; i < n; i++)
    {
        u[i] = IloIntVar(env, 1, n - 1);
    }

    const int M = n - 2;

    for (u_int i = 1; i < n; i++)
    {
        model.add(1 <= u[i]);
        model.add(u[i] <= (int)(n - 1));

        for (u_int j = 1; j < n; j++)
        {
            if (i == j) continue;

            for (u_int k = 0; k < m; k++)
            {
                IloExpr left(env);
                IloExpr right(env);
                left += (u[i] + x[index3(i, j, k)]);
                for (u_int l = k; l < m; l++)
                {
                    right += (u[j] + M * (1 - x[index3(i, j, l)]));
                    model.add(left <= right);
                }
            }
        }
    }
}




