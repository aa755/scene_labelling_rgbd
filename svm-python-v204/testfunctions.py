#! /usr/bin/python

__author__="hema"
__date__ ="$Apr 9, 2011 10:19:41 PM$"
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix

import glpk

def read_examples(filename):
    # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()

    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    # Open the file and read each example.
    for input_file in file(filename):
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());

        max_target=0
        num_node_feats=0
        num_edge_feats=0

        # find the max class and number of node features -- will work for sparse representation
        for i in xrange(0,N):
            target = int(input[i+1][0]);
            if (max_target<int(target)):
                max_target=int(target)
            tokens = [line.split(':') for line in input[i+1][2:]]
            for k,v in tokens:
                if(num_node_feats<int(k)):
                    num_node_feats=int(k)
        for i in xrange(N,N+E):
            tokens = [line.split(':') for line in input[i+1][4:]]
            for k,v in tokens:
                if(num_edge_feats<int(k)):
                    num_edge_feats=int(k)

        print max_target
        print num_node_feats
        print num_edge_feats
        
        Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
        Yn= mat(zeros((max_target*N,1)))
        #XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))
        node_map = {}
        edges = mat(zeros((E,2)))
        for i in xrange(0,N):
            target = int(input[i+1][0]);
            Yn[i*max_target+(target-1),0]=1
            # get the segment number
            node_map[int(input[i+1][1])] = i

            # Get the features.
            tokens = [line.split(':') for line in input[i+1][2:]]
            #print [float(v) for k,v in tokens]
            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            print features
            f = features.transpose()
            #XY_test[(target-1)*num_node_feats:target*num_node_feats,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target):
                #print X[j*9:(j+1)*9,j];
                Xn[j*num_node_feats:(j+1)*num_node_feats,i*max_target+(j)] = f.copy();
            #print X
        Xe = mat(zeros((max_target*max_target*num_edge_feats,max_target*max_target*E)))
        Ye = mat(zeros((max_target*max_target*E,1)))
        for i in xrange(N,N+E):
            target1 = int(input[i+1][0]);
            target2 = int(input[i+1][1]);
            Ye[(i-N)*max_target*max_target + (target1-1)*max_target+(target2-1)]=1
            # get the segment numbers
            edges[i-N,0]= node_map[int(input[i+1][2])]
            edges[i-N,1]= node_map[int(input[i+1][3])]

            tokens = [line.split(':') for line in input[i+1][4:]]
            features = mat([float(v) for k,v in tokens])
            print features
            f = features.transpose()
            z = max_target*num_node_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats
            #XY_test[z : z+num_edge_feats ,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target*max_target):
                #print X[j*9:(j+1)*9,j];
                Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();

        print Xe.shape[0]
        print Xe.shape[1]
        a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye))
        X_s = lil_matrix(X)
        Y_s = lil_matrix(Y)

        # Add the example to the list
        examples.append(((X_s, edges), Y_s))

    # Print out some very useful statistics.
    print len(examples),'examples read'
    return examples

def lp(X,K,N,E):
    edge = X[1]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1






if __name__ == "__main__":
    E= read_examples('data/testfile.txt')
    lp(E[0][0],5,5,4)
    print "Hello World";
