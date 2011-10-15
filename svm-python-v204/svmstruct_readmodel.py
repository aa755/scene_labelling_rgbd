"""A module for SVM^python for multiclass MRF learning."""

# Thomas Finley, tfinley@gmail.com
from numpy import random
import time
from operator import concat
import svmapi, array
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix
from scipy.sparse import csr_matrix
from numpy.ma.core import zeros
import glpk
from bitarray import bitarray

global NUM_CLASSES
global FN
global FE
NUM_CLASSES = 0
FN = 0
FE = 0
 
def read_examples(filename,sparm):
    global NUM_CLASSES
    global FN
    global FE
    print sparm
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
    max_target=0
    num_node_feats=0
    num_edge_feats=0

    # Open the file and read each example.
    for input_file in file(filename):
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());
        K = int(input[0][2].strip());
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
    NUM_CLASSES = K # use the max number of classes read from the file
    FE = num_edge_feats
    FN = num_node_feats
    #print 'number of classes: ', max_target
    #print 'number of node features: ', num_node_feats
    #print 'number of edge features: ',num_edge_feats
    examples.append((1,2));
    return examples
    

def get_index(edges,u,v):
    for i in xrange(0,edges.shape[0]):
        if (edges[i,0] == u and edges[i,1] == v):
            return i
    assert(2 == 1) # should never reach here




def classification_score(x,y,sm,sparm):
    """Return an example, label pair discriminant score."""
    # Utilize the svmapi.Model convenience method 'classify'.
    score = sm.svm_model.classify(psi(x,y,sm,sparm))
    global thecount
    thecount += 1
    if (sum(abs(w) for w in sm.w)):
        import pdb; pdb.set_trace()
    return score

def classify_example(x, sm, sparm):
    """Returns the classification of an example 'x'."""
    print_weight_vector(sm,sparm)
    return ""


    



def write_model(filename, sm, sparm):
    import cPickle, bz2
    cPickle.dump(sm,file(filename,'w'))

def print_weight_vector( sm, sparm):
    global FN
    global FE
    w_list = [sm.w[i] for i in xrange(0,sm.size_psi)]
    #print
    #print sm.size_psi,FN,FE,NUM_CLASSES
    K = NUM_CLASSES
    node_feats_list = [15,51]
    node_coeff = zeros((K,20))
    
    for k in xrange(0,K):
        c = 0;
        for f in node_feats_list:
            node_coeff[k,c*10:(c+1)*10] = w_list[k*FN+f*10: k*FN+(f+1)*10]
            c += 1
    filename="node_weights.csv" 
    savetxt(filename,node_coeff,fmt='%f',delimiter=',');
    edge_feats_list = [5,6,7,10]
    edge_coeff = zeros((K,K,40))
    for n1 in xrange(0,K):
        for n2 in xrange(0,K):
            c = 0;
            for e in edge_feats_list:
                edge_coeff[n1,n2,c*10:(c+1)*10] = w_list[FN*K+ n1*K*FE + n2*FE+ e*10: FN*K+ n1*K*FE + n2*FE + (e+1)*10  ]
                c+=1
            #for e in xrange(0,FE/10):
               # coeff[e,n1,n2] = max(w_list[FN*K+ n1*K*FE + n2*FE + e*10 : FN*K+ n1*K*FE + n2*FE + e*10 + 9 ])
            
    for k in xrange(0,K):
        filename="edge_weights_%d.csv" % k
        savetxt(filename,edge_coeff[k,:,:],fmt='%f',delimiter=',');
    

def read_model(filename, sparm):
    import cPickle, bz2
    return cPickle.load(file(filename))





def print_testing_stats(sample, sm, sparm, teststats):
    return ""

