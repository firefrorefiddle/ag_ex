'''
Created on Jan 11, 2015

@author: smortezapoor
'''

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

def ShowIt(g):
    dt = [('len', float)]
    _A = g['amatrix']
    
    _col = []
    _col.append('gray')
    for i in range(1, g['no_nodes']+1):
        if(g['node_id'][i] == 'D'):
            _col.append('green')
        elif (g['node_id'][i] == 'S'):
            _col.append('red')
    g['node_id'][0] = 'NU'
    
    for i in range(0, len(g['node_id'])):
        g['node_id'][i] = i
    
    G = nx.from_numpy_matrix(_A)
    nx.draw_spring(G, node_color = _col, with_labels = True, labels = g['node_id'] , node_size=600)
    plt.show()
    
    '''_G = nx.from_numpy_matrix(_A)
    print _G
    nx.draw(_G)
    plt.draw()
 #   _G = nx.relabel_nodes(_G, dict(zip(range(len(_G.nodes())),string.ascii_uppercase)))    
    _G = nx.to_agraph(_G)
    _G.node_attr.update(color="red", style="filled")
    _G.edge_attr.update(color="blue", width="2.0")
    
    _G.draw('/tmp/out.png', format='png', prog='neato')'''

def Parse(fileLocation):
    fpoint = open(fileLocation, 'rb')
    lines = fpoint.readlines()
    no_nodes = int(lines[0])
    time_limit = int(lines[1])
    no_vehicles = int(lines[2])
    node_id = dict()
    for i in range(3, 3+no_nodes):
        node_id[int(lines[i].split(' ')[0])] = lines[i].split(' ')[1].replace('\n', '')
  
    strRest = ''
    for i in range(3+no_nodes, 3+no_nodes+no_nodes+1):
        strRest+= lines[i]
              
    nodes = np.fromstring(strRest, dtype=int, sep=' ').reshape((no_nodes+1, no_nodes+1))
    
    for i in range (0, len(node_id)):
        for j in range (0, len(node_id)):
            if i == j:
                continue
            
            if node_id[i+1] == node_id[j+1]:
                nodes[i+1][j+1] = 0
                nodes[j+1][i+1] = 0
    
    obj_return = {}
    obj_return['no_nodes'] = no_nodes
    obj_return['time_limit'] = time_limit
    obj_return['no_vehicles'] = no_vehicles
    obj_return['node_id'] = node_id
    obj_return['amatrix'] = nodes
    
    return obj_return

def nx_graph_from_biadjacency_matrix(M):
    # Give names to the nodes in the two node sets
    U = [ "u{}".format(i) for i in range(M.shape[0]) ]
    V = [ "v{}".format(i) for i in range(M.shape[1]) ]

    # Create the graph and add each set of nodes
    G = nx.Graph()
    G.add_nodes_from(U, bipartite=0)
    G.add_nodes_from(V, bipartite=1)

    # Find the non-zero indices in the biadjacency matrix to connect 
    # those nodes
    G.add_edges_from([ (U[i], V[j]) for i, j in zip(*M.nonzero()) ])

    return G

